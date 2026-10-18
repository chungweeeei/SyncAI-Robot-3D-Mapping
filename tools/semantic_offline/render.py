"""Headless views of a semantic voxel map: top-down rasters (everything labelled, the floor band,
objects on a faint floor) and a legend, for a quick look without CloudCompare. Classes are drawn in
the backend's own palette; the floor / wall / ceiling used to pick the bands are found through
the coarse taxonomy, so the script works for any label space.

    render.py <run dir> [name]      (name: map_semantic | map_semantic_clean, default map_semantic)
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

from pcdio import read_labeled
from taxonomy import COARSE_ID, UNLABELED, coarse_lut

out_dir = Path(sys.argv[1])
name = sys.argv[2] if len(sys.argv) > 2 else "map_semantic"
classes = json.load(open(out_dir / "classes.json"))
pal = np.array(classes["palette"], np.uint8)
id2label = {int(k): v.split(",")[0] for k, v in classes["id2label"].items()}
lut = coarse_lut({int(k): v for k, v in classes["id2label"].items()})

xyz, lab, _ = read_labeled(out_dir / f"{name}.pcd")
lab = lab.astype(np.int64)
coarse = lut[lab]
res = 0.1
mn = xyz[:, :2].min(0)
size = np.ceil((xyz[:, :2].max(0) - mn) / res).astype(int) + 1
print("extent m", xyz.min(0), xyz.max(0), "raster", size)


def raster(mask, fname, order_key):
    img = np.full((size[1], size[0], 3), 25, np.uint8)
    sel = np.nonzero(mask)[0]
    sel = sel[np.argsort(order_key[sel])]  # draw low key first so the high key wins
    ij = ((xyz[sel, :2] - mn) / res).astype(int)
    img[size[1] - 1 - ij[:, 1], ij[:, 0]] = pal[lab[sel]][:, ::-1]
    img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(out_dir / fname), img)
    print("wrote", fname, img.shape)


labelled = lab != UNLABELED
z = xyz[:, 2]
is_floor = coarse == COARSE_ID["floor"]
is_wall = coarse == COARSE_ID["wall"]
is_ceiling = coarse == COARSE_ID["ceiling"]
suffix = "" if name == "map_semantic" else f"_{name.removeprefix('map_semantic_')}"
# 1) everything labelled, lowest z drawn first (ceiling on top)
raster(labelled, f"view_topdown_all{suffix}.jpg", z)
# 2) floor band: everything up to 0.3 m above the floor's 95th percentile, ceiling excluded
z_floor = np.percentile(z[is_floor], 95) if is_floor.any() else np.percentile(z, 10)
raster(labelled & (z < z_floor + 0.3) & ~is_ceiling, f"view_topdown_lowband{suffix}.jpg", -z)
# 3) objects (neither floor, wall nor ceiling) drawn over the floor
objects = labelled & ~(is_floor | is_wall | is_ceiling)
raster(
    labelled & (is_floor | objects),
    f"view_topdown_objects{suffix}.jpg",
    np.where(objects, 10.0, 0.0),
)
# legend of the 18 largest classes
ids, counts = np.unique(lab[labelled], return_counts=True)
top = sorted(zip(counts, ids), reverse=True)[:18]
leg = np.full((26 * len(top) + 10, 320, 3), 255, np.uint8)
for r, (c, i) in enumerate(top):
    y = 10 + 26 * r
    cv2.rectangle(leg, (10, y), (40, y + 18), tuple(int(v) for v in pal[i][::-1]), -1)
    cv2.putText(
        leg, f"{id2label[int(i)]} {c}", (50, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
    )
cv2.imwrite(str(out_dir / f"legend{suffix}.png"), leg)
