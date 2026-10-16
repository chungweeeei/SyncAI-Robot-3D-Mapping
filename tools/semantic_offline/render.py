"""Headless views of the semantic voxel map: a top-down raster (labelled voxels only, floor level
band and everything) and a per-class legend strip, for a quick look without CloudCompare."""

import json
import sys
from pathlib import Path
import cv2
import numpy as np

out_dir = Path(sys.argv[1])
pcd = out_dir / "map_semantic.pcd"
classes = json.load(open(out_dir / "classes.json"))
pal = np.array(classes["palette"], np.uint8)
id2label = {int(k): v.split(",")[0] for k, v in classes["id2label"].items()}
# read label + rgb back from our PCD (fields x y z rgb label confidence)
with open(pcd, "rb") as f:
    while True:
        line = f.readline()
        if line.startswith(b"POINTS"):
            n = int(line.split()[1])
        if line.startswith(b"DATA"):
            break
    arr = np.frombuffer(
        f.read(),
        dtype=np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("rgb", "<f4"),
                ("label", "<u4"),
                ("conf", "<f4"),
            ]
        ),
        count=n,
    )
xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1)
lab = arr["label"].astype(np.int64)
res = 0.1
mn = xyz[:, :2].min(0)
size = np.ceil((xyz[:, :2].max(0) - mn) / res).astype(int) + 1
print("extent m", xyz.min(0), xyz.max(0), "raster", size)


def raster(mask, name, order_key):
    img = np.full((size[1], size[0], 3), 25, np.uint8)
    sel = np.nonzero(mask)[0]
    sel = sel[np.argsort(order_key[sel])]  # draw low z first so high z wins (top-down)
    ij = ((xyz[sel, :2] - mn) / res).astype(int)
    img[size[1] - 1 - ij[:, 1], ij[:, 0]] = pal[lab[sel]][:, ::-1]
    img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(out_dir / name), img)
    print("wrote", name, img.shape)


labelled = lab != 255
z = xyz[:, 2]
# 1) everything labelled, lowest z drawn first (ceiling on top)
raster(labelled, "view_topdown_all.jpg", z)
# 2) floor band only (0.8 m below the lidar path and below): what a traversability map would use
raster(
    labelled & (z < np.percentile(z[lab == 3], 95) + 0.3) & (lab != 5),
    "view_topdown_lowband.jpg",
    -z,
)
# 3) non floor/wall/ceiling objects, on a faint floor
objects = labelled & ~np.isin(lab, [0, 3, 5])
raster(labelled & ((lab == 3) | objects), "view_topdown_objects.jpg", np.where(objects, 10.0, 0.0))
# legend
ids, counts = np.unique(lab[labelled], return_counts=True)
top = sorted(zip(counts, ids), reverse=True)[:18]
leg = np.full((26 * len(top) + 10, 320, 3), 255, np.uint8)
for r, (c, i) in enumerate(top):
    y = 10 + 26 * r
    cv2.rectangle(leg, (10, y), (40, y + 18), tuple(int(v) for v in pal[i][::-1]), -1)
    cv2.putText(
        leg, f"{id2label[int(i)]} {c}", (50, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
    )
cv2.imwrite(str(out_dir / "legend.png"), leg)
