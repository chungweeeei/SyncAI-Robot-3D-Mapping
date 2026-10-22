"""Label every keyframe patch from the image nearest its stamp: segment the image, move the patch
from the keyframe's body pose to the body pose at the image stamp (LIO odometry interpolation),
project through T_cam_body + K/D, keep the nearest point per pixel cell against occlusion, and
write labels/<i>.npz (label uint8 per patch point, 255 = unlabelled; weight = confidence, reduced
on label boundaries)."""

from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from bagio import ImageIndex, body_to_camera, project, read_static, view_cone
from pcdio import read_xyz
from segment import UNLABELED, make_segmenter


def load_keyframes(path: Path):
    kf = []
    for line in path.read_text().splitlines():
        p = line.split()
        if len(p) < 9:
            continue
        kf.append(
            (
                p[0],
                int(p[1]),
                np.array([float(x) for x in p[2:5]]),
                np.array([float(p[6]), float(p[7]), float(p[8]), float(p[5])]),
            )
        )
    return kf


class OdomInterp:
    def __init__(self, path: Path):
        rows = [line.split() for line in path.read_text().splitlines() if line.strip()]
        self.ns = np.array([int(r[0]) for r in rows], dtype=np.int64)
        self.t = np.array([[float(x) for x in r[1:4]] for r in rows])
        q = np.array([[float(r[5]), float(r[6]), float(r[7]), float(r[4])] for r in rows])
        self.rot = R.from_quat(q)
        self.slerp = Slerp(self.ns.astype(np.float64), self.rot)

    def pose(self, t_ns: int):
        """(R, t) of the body in the odom frame at t_ns, clamped to the recorded range."""
        t_ns = int(np.clip(t_ns, self.ns[0], self.ns[-1]))
        i = np.searchsorted(self.ns, t_ns)
        if i < len(self.ns) and self.ns[i] == t_ns:
            return self.rot[i], self.t[i]
        a, b = i - 1, i
        w = (t_ns - self.ns[a]) / (self.ns[b] - self.ns[a])
        return self.slerp([float(t_ns)])[0], (1 - w) * self.t[a] + w * self.t[b]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("map_dir")
    ap.add_argument("keyframes")
    ap.add_argument("odom")
    ap.add_argument("out")
    ap.add_argument("--model", default="facebook/mask2former-swin-large-ade-semantic")
    ap.add_argument("--cell", type=int, default=8, help="z-buffer cell size, px")
    ap.add_argument("--max-image-dt-ms", type=float, default=60.0)
    ap.add_argument("--viz-every", type=int, default=100)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument(
        "--stride", type=int, default=1, help="label every N-th keyframe (for the slow backends)"
    )
    ap.add_argument("--short-side", type=int, default=512, help="segmentation input short side, px")
    a = ap.parse_args()
    bag, map_dir, out = Path(a.bag), Path(a.map_dir), Path(a.out)
    (out / "labels").mkdir(parents=True, exist_ok=True)
    (out / "qa").mkdir(exist_ok=True)

    cam, tfs = read_static(bag)
    T_cam_body = body_to_camera(tfs)
    idx = ImageIndex(bag)
    odom = OdomInterp(Path(a.odom))
    kfs = load_keyframes(Path(a.keyframes))
    if a.limit:
        kfs = kfs[: a.limit]
    # keyframe indices stay the original ones under --stride, so QA images line up across runs
    kf_index = list(range(len(kfs)))[:: a.stride]
    kfs = kfs[:: a.stride]
    seg = make_segmenter(a.model, short_side=a.short_side)
    json.dump(
        {"id2label": seg.id2label, "palette": seg.palette.tolist(), "model": a.model},
        open(out / "classes.json", "w"),
    )

    # image index -> keyframes that use it
    by_image = defaultdict(list)
    skipped = 0
    for k, (name, st, _, _) in zip(kf_index, kfs):
        j, dt = idx.nearest(st)
        if dt > a.max_image_dt_ms * 1e6:
            skipped += 1
            continue
        by_image[j].append((k, name, st, dt))
    print(
        f"{len(kfs)} keyframes -> {len(by_image)} images ({skipped} without an image within {a.max_image_dt_ms} ms)"
    )

    W, H = cam.width, cam.height
    t0 = time.time()
    done = 0
    seg_s = 0.0
    stats = []
    for j, img_stamp, img in idx.read_many(sorted(by_image)):
        ts = time.time()
        lab, conf, bnd = seg(img)
        seg_s += time.time() - ts
        for k, name, st, dt in by_image[j]:
            xyz = read_xyz(map_dir / "patches" / name)
            # patch (body @ keyframe stamp) -> body @ image stamp, through the odom frame
            R_kf, t_kf = odom.pose(st)
            R_im, t_im = odom.pose(img_stamp)
            p_odom = R_kf.apply(xyz) + t_kf
            p_body_im = R_im.inv().apply(p_odom - t_im)
            p_cam = T_cam_body.apply(p_body_im)
            labels = np.full(len(xyz), UNLABELED, np.uint8)
            weights = np.zeros(len(xyz), np.float32)
            z = p_cam[:, 2]
            cone = view_cone(cam, p_cam)
            ci = np.nonzero(cone)[0]
            if len(ci):
                uv = project(cam, p_cam[ci])
                u, v = uv[:, 0], uv[:, 1]
                inside = (u >= 2) & (u < W - 2) & (v >= 2) & (v < H - 2)
                ci, u, v, zc = ci[inside], u[inside], v[inside], z[ci][inside]
            # a patch whose cone points all project past the image border (dp1f_1019_with_image
            # had one) leaves nothing here, and the run-start mask below needs at least one point
            if len(ci):
                # occlusion: keep points within 10 % (min 0.3 m) of the nearest point of their cell
                cell = (v.astype(int) // a.cell) * (W // a.cell + 1) + (u.astype(int) // a.cell)
                order = np.lexsort((zc, cell))
                cell_s, z_s = cell[order], zc[order]
                first = np.r_[True, cell_s[1:] != cell_s[:-1]]
                zmin = np.repeat(z_s[first], np.diff(np.r_[np.nonzero(first)[0], len(cell_s)]))
                keep = z_s <= zmin + np.maximum(0.3, 0.1 * zmin)
                sel = order[keep]
                ui, vi = u[sel].astype(int), v[sel].astype(int)
                labels[ci[sel]] = lab[vi, ui]
                weights[ci[sel]] = conf[vi, ui] * np.where(bnd[vi, ui], 0.3, 1.0)
            np.savez_compressed(
                out / "labels" / f"{Path(name).stem}.npz",
                label=labels,
                weight=weights,
                image=j,
                dt_ns=dt,
            )
            frac = (labels != UNLABELED).mean()
            stats.append((k, j, dt / 1e6, frac))
            if a.viz_every and k % a.viz_every == 0:
                vis = cv2.addWeighted(img, 0.6, seg.colorize(lab), 0.4, 0)
                if len(ci):
                    cols = seg.palette[labels[ci[sel]]][:, ::-1]
                    for (uu, vv), c in zip(zip(ui, vi), cols):
                        cv2.circle(vis, (int(uu), int(vv)), 2, tuple(int(x) for x in c), -1)
                cv2.putText(
                    vis,
                    f"kf {k} img {j} dt {dt / 1e6:.0f} ms labelled {frac:.0%}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                )
                cv2.imwrite(
                    str(out / "qa" / f"kf_{k:05d}.jpg"), vis, [cv2.IMWRITE_JPEG_QUALITY, 85]
                )
            done += 1
        if done % 200 < len(by_image[j]):
            el = time.time() - t0
            print(
                f"  {done}/{len(kfs)} keyframes, {el:.0f} s, {el / max(done, 1):.2f} s/kf",
                flush=True,
            )
    fr = np.array([s[3] for s in stats])
    dts = np.array([abs(s[2]) for s in stats])
    print(
        f"done {done} keyframes in {time.time() - t0:.0f} s; labelled fraction per patch mean {fr.mean():.2f} "
        f"(min {fr.min():.2f}); image |dt| mean {dts.mean():.1f} ms max {dts.max():.1f} ms"
    )
    with open(out / "label_stats.txt", "w") as f:
        for s in stats:
            f.write(f"{s[0]} {s[1]} {s[2]:.1f} {s[3]:.3f}\n")
    json.dump(
        {
            "model": a.model,
            "device": seg.device,
            "short_side": a.short_side,
            "stride": a.stride,
            "keyframes": done,
            "images": len(by_image),
            "seconds_total": time.time() - t0,
            "seconds_segmentation": seg_s,
            "seconds_per_image": seg_s / max(len(by_image), 1),
            "labelled_fraction_mean": float(fr.mean()),
            "image_dt_ms_mean": float(dts.mean()),
            "image_dt_ms_max": float(dts.max()),
        },
        open(out / "run.json", "w"),
        indent=1,
    )


if __name__ == "__main__":
    main()
