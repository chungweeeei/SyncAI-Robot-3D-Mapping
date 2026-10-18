"""Text-prompted 2D instance segmentation for the instance-level object map (`instances.py`).
Every backend returns, per image, a list of (mask HxW bool, label str, score float), one entry per
instance, with `label` one of the prompts.

- `sam3`: Meta's SAM 3 through transformers (`Sam3Model`). One prompt per forward pass on shared
  vision features, so the cost grows with the prompt list. The checkpoint `facebook/sam3` is
  gated (manual approval): request access on its model page, then `hf auth login`. NOT TESTED
  against the weights yet: written to the transformers 5.20 API, which this file was the first
  to need; check one image with `seg_test`-style output before a full run.
- `grounded-sam`: Grounding DINO tiny (all prompts in one pass, boxes) + SAM ViT-B (a mask per
  box). Ungated, and the stand-in used to validate the association while SAM 3 was out of reach.
"""

from __future__ import annotations

import numpy as np
import torch

# Things worth finding in a building, the coarse groups the object map clusters. Surfaces and
# passers-by (floor, wall, ceiling, window, column, person) are left out on purpose.
OBJECT_PROMPTS = [
    "sofa",
    "armchair",
    "chair",
    "table",
    "shelf",
    "cabinet",
    "counter",
    "door",
    "potted plant",
    "trash can",
    "signboard",
    "screen",
    "escalator",
    "stairs",
    "box",
    "bench",
    "fire extinguisher",
    "vending machine",
]


def _device():
    return "mps" if torch.backends.mps.is_available() else "cpu"


class GroundedSamInstances:
    name = "grounded-sam"

    def __init__(
        self,
        prompts=None,
        dino="IDEA-Research/grounding-dino-tiny",
        sam="facebook/sam-vit-base",
        box_threshold=0.3,
        text_threshold=0.25,
    ):
        from transformers import (
            AutoModelForZeroShotObjectDetection,
            AutoProcessor,
            SamModel,
            SamProcessor,
        )

        self.device = _device()
        self.prompts = list(prompts or OBJECT_PROMPTS)
        self.text = ". ".join(self.prompts) + "."
        self.box_threshold, self.text_threshold = box_threshold, text_threshold
        self.dino_proc = AutoProcessor.from_pretrained(dino)
        self.dino = (
            AutoModelForZeroShotObjectDetection.from_pretrained(dino).float().to(self.device).eval()
        )
        self.sam_proc = SamProcessor.from_pretrained(sam)
        self.sam = SamModel.from_pretrained(sam).float().to(self.device).eval()

    def _prompt_of(self, text: str):
        text = text.strip().lower()
        if text in self.prompts:
            return text
        # Grounding DINO returns the matched span; "potted plant" can come back as "plant",
        # "sofa armchair" as both. Longest prompt contained in the span wins.
        hits = [p for p in self.prompts if p in text or text in p]
        return max(hits, key=len) if hits else None

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
        dets = []
        for b, s, t in zip(
            res["boxes"].cpu().numpy(),
            res["scores"].cpu().numpy(),
            res.get("text_labels", res.get("labels")),
        ):
            lab = self._prompt_of(str(t))
            # a box over most of the frame is a scene-level hit ("table" for a whole food court)
            if lab and (b[2] - b[0]) * (b[3] - b[1]) < 0.7 * W * H:
                dets.append((b, float(s), lab))
        if not dets:
            return []
        sam_in = self.sam_proc(
            pil, input_boxes=[[list(map(float, d[0])) for d in dets]], return_tensors="pt"
        )
        sam_in = {
            k: (v.float() if v.dtype == torch.float64 else v).to(self.device)
            for k, v in sam_in.items()
        }
        sam_out = self.sam(**sam_in, multimask_output=False)
        masks = self.sam_proc.image_processor.post_process_masks(
            sam_out.pred_masks.cpu(),
            sam_in["original_sizes"].cpu(),
            sam_in["reshaped_input_sizes"].cpu(),
        )[0][:, 0].numpy()
        return [(m, d[2], d[1]) for m, d in zip(masks, dets) if m.any()]


class Sam3Instances:
    name = "sam3"

    def __init__(self, prompts=None, model="facebook/sam3", threshold=0.5, mask_threshold=0.5):
        from transformers import Sam3Model, Sam3Processor

        self.device = _device()
        self.prompts = list(prompts or OBJECT_PROMPTS)
        self.threshold, self.mask_threshold = threshold, mask_threshold
        self.proc = Sam3Processor.from_pretrained(model)
        self.model = Sam3Model.from_pretrained(model).float().to(self.device).eval()

    @torch.no_grad()
    def __call__(self, img_bgr: np.ndarray):
        from PIL import Image

        H, W = img_bgr.shape[:2]
        pil = Image.fromarray(np.ascontiguousarray(img_bgr[:, :, ::-1]))
        img_in = self.proc(images=pil, return_tensors="pt").to(self.device)
        # the image encoder is most of the cost: run it once and reuse it for every prompt
        vision = self.model.get_vision_features(pixel_values=img_in.pixel_values)
        out = []
        for prompt in self.prompts:
            txt = self.proc(text=prompt, return_tensors="pt").to(self.device)
            o = self.model(
                vision_embeds=vision, input_ids=txt.input_ids, attention_mask=txt.attention_mask
            )
            res = self.proc.post_process_instance_segmentation(
                o,
                threshold=self.threshold,
                mask_threshold=self.mask_threshold,
                target_sizes=[(H, W)],
            )[0]
            for m, s in zip(res["masks"].cpu().numpy(), res["scores"].cpu().numpy()):
                m = m.astype(bool)
                if m.any() and m.mean() < 0.7:
                    out.append((m, prompt, float(s)))
        return out


def make_instance_segmenter(name: str, **kw):
    if name == "sam3":
        return Sam3Instances(**kw)
    if name == "grounded-sam":
        return GroundedSamInstances(**kw)
    raise ValueError(f"unknown instance backend {name!r} (sam3 | grounded-sam)")
