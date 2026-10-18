"""2D semantic segmentation backends for the offline labeller. Every backend has the same output,
per pixel at full image resolution: class id (uint8, 255 = no label), confidence in [0, 1] and a
label-boundary mask, plus `id2label` / `palette`, so `label_patches.py` and `fuse.py` never see
which model produced the labels.

Backends (picked by `make_segmenter(name)` from the checkpoint name):

- Mask2Former (`facebook/mask2former-*`): semantic *and* panoptic checkpoints (the COCO panoptic
  one gives a 133-class semantic map through the same query x mask product).
- OneFormer (`shi-labs/oneformer_*`): same output head, a task token on the input.
- SegFormer (`nvidia/segformer-*`): per-pixel logits at 1/4 resolution; the lightest, the one
  that would run on the Jetson.
- Grounding DINO + SAM (`grounded-sam`, open vocabulary): text prompts -> boxes -> SAM masks,
  painted largest first so objects overwrite the stuff they stand on. Slow, and the only one that
  leaves pixels unlabelled.
"""

from __future__ import annotations

import numpy as np
import torch
import torch.nn.functional as F

UNLABELED = 255

# Fixed colours for the classes the map cares about (matched on the first comma-separated name);
# everything else gets a seeded random colour.
KEY_COLORS = {
    "floor": (80, 200, 80),
    "wall": (160, 160, 160),
    "ceiling": (120, 170, 230),
    "door": (160, 90, 30),
    "person": (255, 0, 0),
    "stairs": (255, 140, 0),
    "stairway": (255, 160, 40),
    "step": (255, 120, 0),
    "grass": (0, 150, 0),
    "road": (90, 90, 90),
    "sidewalk": (190, 190, 150),
    "table": (200, 120, 200),
    "chair": (240, 200, 60),
    "column": (200, 200, 220),
    "railing": (200, 60, 120),
    "bannister": (200, 60, 160),
    "windowpane": (100, 220, 255),
    "glass": (120, 230, 255),
    "counter": (210, 160, 90),
    "cabinet": (150, 110, 60),
    "escalator": (255, 80, 200),
    "signboard": (255, 255, 0),
    "plant": (60, 180, 120),
    "tree": (30, 130, 60),
    "sky": (150, 200, 255),
    "building": (120, 100, 100),
    "box": (220, 180, 140),
    "rug": (170, 140, 90),
    "light": (255, 255, 200),
    "mirror": (200, 240, 255),
    "bar": (180, 120, 60),
    "shelf": (140, 100, 70),
    # COCO panoptic spellings
    "floor-other-merged": (80, 200, 80),
    "floor-wood": (80, 200, 80),
    "wall-other-merged": (160, 160, 160),
    "wall-brick": (160, 160, 160),
    "wall-stone": (160, 160, 160),
    "wall-tile": (160, 160, 160),
    "wall-wood": (160, 160, 160),
    "ceiling-merged": (120, 170, 230),
    "door-stuff": (160, 90, 30),
    "window-other": (100, 220, 255),
    "couch": (240, 200, 60),
    "dining table": (200, 120, 200),
    "table-merged": (200, 120, 200),
    "cabinet-merged": (150, 110, 60),
    "potted plant": (60, 180, 120),
}

# Prompts for the open-vocabulary backend: the coarse classes the map is for, in Grounding DINO's
# "a. b. c." syntax. Order matters only for the palette.
GROUNDED_PROMPTS = [
    "floor",
    "wall",
    "ceiling",
    "door",
    "person",
    "stairs",
    "chair",
    "table",
    "column",
    "window",
    "shelf",
    "sofa",
    "counter",
    "signboard",
    "plant",
    "escalator",
    "light",
    "box",
]


def palette(id2label: dict[int, str]) -> np.ndarray:
    rng = np.random.default_rng(7)
    pal = np.zeros((256, 3), np.uint8)
    for i in range(len(id2label)):
        name = id2label[i].split(",")[0].strip()
        pal[i] = KEY_COLORS.get(name, tuple(int(x) for x in rng.integers(40, 230, 3)))
    pal[UNLABELED] = (30, 30, 30)
    return pal


def boundary_mask(lab: np.ndarray, r: int = 2) -> np.ndarray:
    """True where any neighbour within r px carries another label (or no label)."""
    H, W = lab.shape
    pad = np.pad(lab, r, mode="edge")
    b = np.zeros((H, W), bool)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dy or dx:
                b |= pad[r + dy : r + dy + H, r + dx : r + dx + W] != lab
    return b


def default_device() -> str:
    return "mps" if torch.backends.mps.is_available() else "cpu"


class SegmenterBase:
    """Shared plumbing: resize to `short_side` (multiple of 32), run, upsample back to H x W."""

    model_name: str
    id2label: dict[int, str]
    palette: np.ndarray
    device: str

    def _size(self, H: int, W: int, short_side: int) -> tuple[int, int]:
        s = short_side / min(H, W)
        return int(round(H * s / 32) * 32), int(round(W * s / 32) * 32)

    def _finish(self, seg: torch.Tensor, H: int, W: int):
        """seg: C x h x w class scores (probabilities). -> (label, conf, boundary) at H x W."""
        conf, label = seg.max(0)
        label = F.interpolate(label[None, None].float(), size=(H, W), mode="nearest")[0, 0].to(
            torch.uint8
        )
        conf = F.interpolate(conf[None, None], size=(H, W), mode="bilinear", align_corners=False)[
            0, 0
        ].clamp(0, 1)
        lab = label.cpu().numpy()
        return lab, conf.cpu().numpy().astype(np.float32), boundary_mask(lab)

    def colorize(self, label: np.ndarray) -> np.ndarray:
        return self.palette[label][:, :, ::-1].copy()  # BGR for cv2


class Mask2FormerSegmenter(SegmenterBase):
    def __init__(self, model_name, device=None, short_side=512):
        from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

        self.model_name, self.device, self.short_side = (
            model_name,
            device or default_device(),
            short_side,
        )
        self.processor = AutoImageProcessor.from_pretrained(model_name)
        self.model = (
            Mask2FormerForUniversalSegmentation.from_pretrained(model_name)
            .float()
            .to(self.device)
            .eval()
        )
        self.id2label = {int(k): v for k, v in self.model.config.id2label.items()}
        self.palette = palette(self.id2label)

    @torch.no_grad()
    def __call__(self, img_bgr: np.ndarray):
        H, W = img_bgr.shape[:2]
        h, w = self._size(H, W, self.short_side)
        rgb = np.ascontiguousarray(img_bgr[:, :, ::-1])
        inputs = self.processor(
            images=rgb, size={"height": h, "width": w}, do_resize=True, return_tensors="pt"
        ).to(self.device)
        out = self.model(**inputs)
        cls = out.class_queries_logits.softmax(-1)[0, :, :-1]  # Q x C (drop "no object")
        masks = out.masks_queries_logits[0].sigmoid()  # Q x h' x w'
        masks = F.interpolate(masks[None], size=(h, w), mode="bilinear", align_corners=False)[0]
        seg = torch.einsum("qc,qhw->chw", cls, masks)
        return self._finish(seg, H, W)


class OneFormerSegmenter(SegmenterBase):
    def __init__(self, model_name, device=None, short_side=512):
        from transformers import OneFormerForUniversalSegmentation, OneFormerProcessor

        self.model_name, self.device, self.short_side = (
            model_name,
            device or default_device(),
            short_side,
        )
        self.processor = OneFormerProcessor.from_pretrained(model_name)
        self.model = (
            OneFormerForUniversalSegmentation.from_pretrained(model_name)
            .float()
            .to(self.device)
            .eval()
        )
        self.id2label = {int(k): v for k, v in self.model.config.id2label.items()}
        self.palette = palette(self.id2label)

    @torch.no_grad()
    def __call__(self, img_bgr: np.ndarray):
        H, W = img_bgr.shape[:2]
        h, w = self._size(H, W, self.short_side)
        rgb = np.ascontiguousarray(img_bgr[:, :, ::-1])
        inputs = self.processor(
            images=rgb,
            task_inputs=["semantic"],
            size={"height": h, "width": w},
            do_resize=True,
            return_tensors="pt",
        ).to(self.device)
        out = self.model(**inputs)
        cls = out.class_queries_logits.softmax(-1)[0, :, :-1]
        masks = out.masks_queries_logits[0].sigmoid()
        masks = F.interpolate(masks[None], size=(h, w), mode="bilinear", align_corners=False)[0]
        seg = torch.einsum("qc,qhw->chw", cls, masks)
        return self._finish(seg, H, W)


class SegFormerSegmenter(SegmenterBase):
    def __init__(self, model_name, device=None, short_side=512):
        from transformers import AutoImageProcessor, SegformerForSemanticSegmentation

        self.model_name, self.device, self.short_side = (
            model_name,
            device or default_device(),
            short_side,
        )
        self.processor = AutoImageProcessor.from_pretrained(model_name)
        self.model = (
            SegformerForSemanticSegmentation.from_pretrained(model_name)
            .float()
            .to(self.device)
            .eval()
        )
        self.id2label = {int(k): v for k, v in self.model.config.id2label.items()}
        self.palette = palette(self.id2label)

    @torch.no_grad()
    def __call__(self, img_bgr: np.ndarray):
        H, W = img_bgr.shape[:2]
        h, w = self._size(H, W, self.short_side)
        rgb = np.ascontiguousarray(img_bgr[:, :, ::-1])
        inputs = self.processor(
            images=rgb, size={"height": h, "width": w}, do_resize=True, return_tensors="pt"
        ).to(self.device)
        logits = self.model(**inputs).logits[0]  # C x h/4 x w/4
        seg = F.interpolate(logits[None], size=(h, w), mode="bilinear", align_corners=False)[0]
        return self._finish(seg.softmax(0), H, W)


class GroundedSamSegmenter(SegmenterBase):
    """Grounding DINO (boxes from text) + SAM (a mask per box). Pixels no mask covers stay 255.
    Masks are painted in order of decreasing area, so a chair's mask overwrites the floor box it
    sits in; a pixel's confidence is its detection score."""

    def __init__(
        self,
        model_name="grounded-sam",
        device=None,
        short_side=0,
        dino="IDEA-Research/grounding-dino-tiny",
        sam="facebook/sam-vit-base",
        prompts=None,
        box_threshold=0.3,
        text_threshold=0.25,
    ):
        from transformers import (
            AutoModelForZeroShotObjectDetection,
            AutoProcessor,
            SamModel,
            SamProcessor,
        )

        self.model_name = model_name
        self.device = device or default_device()
        self.prompts = list(prompts or GROUNDED_PROMPTS)
        self.text = ". ".join(self.prompts) + "."
        self.box_threshold, self.text_threshold = box_threshold, text_threshold
        self.dino_proc = AutoProcessor.from_pretrained(dino)
        self.dino = (
            AutoModelForZeroShotObjectDetection.from_pretrained(dino).float().to(self.device).eval()
        )
        self.sam_proc = SamProcessor.from_pretrained(sam)
        self.sam = SamModel.from_pretrained(sam).float().to(self.device).eval()
        self.id2label = {i: p for i, p in enumerate(self.prompts)}
        self.palette = palette(self.id2label)

    def _label_id(self, text: str) -> int | None:
        """Grounding DINO returns the matched phrase; map it back onto one prompt."""
        text = text.strip().lower()
        if text in self.id2label.values():
            return self.prompts.index(text)
        for i, p in enumerate(self.prompts):  # partial matches ("dining table" -> table)
            if p in text or text in p:
                return i
        return None

    @torch.no_grad()
    def __call__(self, img_bgr: np.ndarray):
        from PIL import Image

        H, W = img_bgr.shape[:2]
        pil = Image.fromarray(np.ascontiguousarray(img_bgr[:, :, ::-1]))
        inputs = self.dino_proc(images=pil, text=self.text, return_tensors="pt").to(self.device)
        out = self.dino(**inputs)
        res = self.dino_proc.post_process_grounded_object_detection(
            out,
            inputs.input_ids,
            threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            target_sizes=[(H, W)],
        )[0]
        boxes = res["boxes"].cpu().numpy()
        scores = res["scores"].cpu().numpy()
        texts = res.get("text_labels", res.get("labels"))
        lab = np.full((H, W), UNLABELED, np.uint8)
        conf = np.zeros((H, W), np.float32)
        dets = []
        for b, s, t in zip(boxes, scores, texts):
            li = self._label_id(str(t))
            if li is not None:
                dets.append((b, float(s), li))
        if dets:
            sam_in = self.sam_proc(
                pil, input_boxes=[[list(map(float, d[0])) for d in dets]], return_tensors="pt"
            )
            # the processor emits float64 boxes; MPS has no float64
            sam_in = {
                k: (v.float() if v.dtype == torch.float64 else v).to(self.device)
                for k, v in sam_in.items()
            }
            sam_out = self.sam(**sam_in, multimask_output=False)
            masks = self.sam_proc.image_processor.post_process_masks(
                sam_out.pred_masks.cpu(),
                sam_in["original_sizes"].cpu(),
                sam_in["reshaped_input_sizes"].cpu(),
            )[0][:, 0].numpy()  # N x H x W bool
            order = np.argsort(-masks.reshape(len(dets), -1).sum(1))  # largest first
            for i in order:
                m = masks[i]
                lab[m] = dets[i][2]
                conf[m] = dets[i][1]
        return lab, conf, boundary_mask(lab)


def make_segmenter(name: str, **kw) -> SegmenterBase:
    low = name.lower()
    if low.startswith("grounded-sam") or low == "grounded_sam":
        return GroundedSamSegmenter(name, **kw)
    if "oneformer" in low:
        return OneFormerSegmenter(name, **kw)
    if "segformer" in low:
        return SegFormerSegmenter(name, **kw)
    return Mask2FormerSegmenter(name, **kw)


# Backwards-compatible name: the first version of this file had one class.
Segmenter = make_segmenter


def model_tag(name: str) -> str:
    """Short directory-friendly name: facebook/mask2former-swin-large-ade-semantic -> m2f-swin-large-ade."""
    low = name.lower().split("/")[-1]
    low = (
        low.replace("mask2former", "m2f").replace("segformer", "segf").replace("oneformer", "onef")
    )
    for drop in ("-semantic", "-finetuned", "_ade20k", "-panoptic"):
        low = low.replace(drop, "")
    return low.replace("_", "-")
