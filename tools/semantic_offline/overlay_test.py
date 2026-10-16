"""Project one Livox scan onto the nearest image with the bag's camera_info + tf_static, to check the
provisional extrinsic before any labelling."""

import sys
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from bagio import ImageIndex, body_to_camera, iter_scans, project, read_static

bag = Path(sys.argv[1])
out = Path(sys.argv[2])
out.mkdir(exist_ok=True)
cam, tfs = read_static(bag)
print("camera", cam.width, cam.height, "\nK", cam.K, "\nD", cam.D)
for k, v in tfs.items():
    if "camera" in k[1] or "lidar_top" in k[1]:
        print(k, "t", v.t, "rpy", R.from_matrix(v.R).as_euler("xyz"))
T_cam_body = body_to_camera(tfs)
T_cam_lidar = tfs[("robot01/lidar_top", "robot01/camera_optical_frame")].inv()
print("T_cam_lidar t", T_cam_lidar.t, "R\n", T_cam_lidar.R)

idx = ImageIndex(bag)
print("images", len(idx.stamps))
t0 = idx.stamps[0]
# a few moments through the bag, offsets in seconds
offsets = [float(s) for s in sys.argv[3:]] or [30, 90, 150, 200]
targets = [t0 + int(o * 1e9) for o in offsets]
scans = {}
for tgt in targets:
    for st, xyz, off, refl in iter_scans(bag, tgt - int(0.2e9), tgt + int(0.3e9)):
        if st >= tgt - int(0.1e9):
            scans[tgt] = (st, xyz, off, refl)
            break
imgs = {}
pairs = {}
for tgt, (st, xyz, off, refl) in scans.items():
    mid = st + int(off.mean())
    j, dt = idx.nearest(mid)
    pairs[tgt] = (j, dt, st, xyz, refl)
for j, stamp, img in idx.read_many([p[0] for p in pairs.values()]):
    imgs[j] = img
for tgt, (j, dt, st, xyz, refl) in pairs.items():
    img = imgs[j].copy()
    p_cam = T_cam_lidar.apply(xyz)
    front = p_cam[:, 2] > 0.3
    uv = project(cam, p_cam[front])
    d = p_cam[front, 2]
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < cam.width) & (uv[:, 1] >= 0) & (uv[:, 1] < cam.height)
    uv, d = uv[inside], d[inside]
    order = np.argsort(-d)  # far first, near drawn on top
    cmap = cv2.applyColorMap(
        (np.clip(d / 8.0, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_JET
    ).reshape(-1, 3)
    for (u, v), c in zip(uv[order].astype(int), cmap[order]):
        cv2.circle(img, (int(u), int(v)), 2, tuple(int(x) for x in c), -1)
    name = out / f"overlay_{(tgt - t0) / 1e9:.0f}s.jpg"
    cv2.imwrite(str(name), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(
        f"{name.name}: scan {st} image {j} dt {dt / 1e6:.1f} ms, {inside.sum()} / {len(xyz)} points in view"
    )
