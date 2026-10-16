"""2D semantic segmentation (Mask2Former, ADE20K 150 classes) for the offline labeller: per-pixel
class id, confidence and a label-boundary mask, at full image resolution."""

from __future__ import annotations

import numpy as np
import torch
import torch.nn.functional as F
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation

UNLABELED = 255

# Fixed colours for the classes the map cares about; everything else gets a seeded random colour.
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
}


def palette(id2label: dict[int, str]) -> np.ndarray:
    rng = np.random.default_rng(7)
    pal = np.zeros((256, 3), np.uint8)
    for i in range(len(id2label)):
        name = id2label[i].split(",")[0].strip()
        pal[i] = KEY_COLORS.get(name, tuple(int(x) for x in rng.integers(40, 230, 3)))
    pal[UNLABELED] = (30, 30, 30)
    return pal


class Segmenter:
    def __init__(
        self, model_name="facebook/mask2former-swin-large-ade-semantic", device=None, short_side=512
    ):
        self.device = device or ("mps" if torch.backends.mps.is_available() else "cpu")
        self.processor = AutoImageProcessor.from_pretrained(model_name)
        self.model = (
            Mask2FormerForUniversalSegmentation.from_pretrained(model_name).to(self.device).eval()
        )
        self.short_side = short_side
        self.id2label = {int(k): v for k, v in self.model.config.id2label.items()}
        self.palette = palette(self.id2label)

    @torch.no_grad()
    def __call__(self, img_bgr: np.ndarray):
        """-> (label HxW uint8, confidence HxW float32 in [0,1], boundary HxW bool)."""
        H, W = img_bgr.shape[:2]
        rgb = np.ascontiguousarray(img_bgr[:, :, ::-1])
        s = self.short_side / min(H, W)
        size = {"height": int(round(H * s / 32) * 32), "width": int(round(W * s / 32) * 32)}
        inputs = self.processor(images=rgb, size=size, do_resize=True, return_tensors="pt").to(
            self.device
        )
        out = self.model(**inputs)
        cls = out.class_queries_logits.softmax(-1)[0, :, :-1]  # Q x C (drop "no object")
        masks = out.masks_queries_logits[0].sigmoid()  # Q x h x w
        masks = F.interpolate(
            masks[None], size=(size["height"], size["width"]), mode="bilinear", align_corners=False
        )[0]
        seg = torch.einsum("qc,qhw->chw", cls, masks)  # C x h x w
        conf, label = seg.max(0)
        # back to the image resolution
        label = F.interpolate(label[None, None].float(), size=(H, W), mode="nearest")[0, 0].to(
            torch.uint8
        )
        conf = F.interpolate(conf[None, None], size=(H, W), mode="bilinear", align_corners=False)[
            0, 0
        ]
        conf = conf.clamp(0, 1)
        lab = label.cpu().numpy()
        # a pixel is on a boundary when any neighbour within 2 px has another label
        pad = np.pad(lab, 2, mode="edge")
        boundary = np.zeros((H, W), bool)
        for dy in (-2, -1, 0, 1, 2):
            for dx in (-2, -1, 0, 1, 2):
                if dy or dx:
                    boundary |= pad[2 + dy : 2 + dy + H, 2 + dx : 2 + dx + W] != lab
        return lab, conf.cpu().numpy().astype(np.float32), boundary

    def colorize(self, label: np.ndarray) -> np.ndarray:
        return self.palette[label][:, :, ::-1].copy()  # BGR for cv2
