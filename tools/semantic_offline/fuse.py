"""Fuse the labelled patches with the corrected poses.txt into a semantic voxel map: per voxel a
weighted class histogram, argmax, written as a coloured PCD/PLY (voxel centroids) with a label
field, plus stats.json. Unlabelled voxels (never in the camera's view) are kept in dark grey so the
map stays readable; drop them with --labelled-only."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from keyframes import load_poses_txt
from pcdio import read_xyz, write_labeled
from segment import UNLABELED


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_dir")
    ap.add_argument("labels_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument(
        "--min-weight", type=float, default=0.5, help="total vote weight a voxel needs for a label"
    )
    ap.add_argument("--labelled-only", action="store_true")
    ap.add_argument(
        "--mask-with",
        default="",
        help="keep only voxels that still hold a point of this PCD (e.g. the cleaned map.pcd)",
    )
    ap.add_argument("--name", default="map_semantic")
    a = ap.parse_args()
    map_dir, labels_dir, out = Path(a.map_dir), Path(a.labels_dir), Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    classes = json.load(open(labels_dir / "classes.json"))
    id2label = {int(k): v.split(",")[0].strip() for k, v in classes["id2label"].items()}
    palette = np.array(classes["palette"], np.uint8)

    names, kt, kq = load_poses_txt(map_dir / "poses.txt")
    keys_all, lab_all, w_all, xyz_all, key_pts = [], [], [], [], []
    n_pts = 0
    for name, t, q in zip(names, kt, kq):
        f = labels_dir / "labels" / f"{Path(name).stem}.npz"
        xyz = read_xyz(map_dir / "patches" / name).astype(np.float64)
        world = R.from_quat(q).apply(xyz) + t
        key = np.floor(world / a.voxel).astype(np.int64)
        # 17 bits per axis (+-6.5 km at 0.1 m), 51 bits in all, so key * 256 + class still fits int64
        key = (
            (key[:, 0] + (1 << 16)) * (1 << 34)
            + (key[:, 1] + (1 << 16)) * (1 << 17)
            + (key[:, 2] + (1 << 16))
        )
        n_pts += len(xyz)
        xyz_all.append(world.astype(np.float32))
        key_pts.append(key)
        if f.exists():
            d = np.load(f)
            lab, w = d["label"], d["weight"]
            m = (lab != UNLABELED) & (w > 0)
            keys_all.append(key[m])
            lab_all.append(lab[m])
            w_all.append(w[m])
    keys = np.concatenate(keys_all)
    labs = np.concatenate(lab_all).astype(np.int64)
    ws = np.concatenate(w_all).astype(np.float64)
    print(f"{len(names)} patches, {n_pts} points, {len(keys)} labelled point observations")

    # per (voxel, class) weight, then argmax per voxel
    kl = keys * 256 + labs
    ukl, inv = np.unique(kl, return_inverse=True)
    wsum = np.bincount(inv, weights=ws)
    vox = ukl // 256
    cls = (ukl % 256).astype(np.uint8)
    order = np.lexsort((-wsum, vox))
    vox_s, cls_s, w_s = vox[order], cls[order], wsum[order]
    first = np.r_[True, vox_s[1:] != vox_s[:-1]]
    vox_u, best_cls, best_w = vox_s[first], cls_s[first], w_s[first]
    total_w = np.add.reduceat(w_s, np.nonzero(first)[0])
    ok = total_w >= a.min_weight
    vox_u, best_cls, best_w, total_w = vox_u[ok], best_cls[ok], best_w[ok], total_w[ok]
    label_of = dict(zip(vox_u.tolist(), best_cls.tolist()))
    conf_of = dict(zip(vox_u.tolist(), (best_w / total_w).tolist()))

    # voxel centroids over ALL points (geometry), labelled or not
    allk = np.concatenate(key_pts)
    allp = np.concatenate(xyz_all)
    uk, inv2 = np.unique(allk, return_inverse=True)
    cnt = np.bincount(inv2)
    cen = np.stack([np.bincount(inv2, weights=allp[:, i]) for i in range(3)], 1) / cnt[:, None]
    lab_v = np.array([label_of.get(int(k), UNLABELED) for k in uk], np.uint8)
    conf_v = np.array([conf_of.get(int(k), 0.0) for k in uk], np.float32)
    if a.mask_with:
        mp = read_xyz(Path(a.mask_with)).astype(np.float64)
        mk = np.floor(mp / a.voxel).astype(np.int64)
        mk = (
            (mk[:, 0] + (1 << 16)) * (1 << 34)
            + (mk[:, 1] + (1 << 16)) * (1 << 17)
            + (mk[:, 2] + (1 << 16))
        )
        m = np.isin(uk, np.unique(mk))
        print(f"mask {a.mask_with}: {len(mp)} points, keeping {m.sum()} of {len(uk)} voxels")
        uk, cen, lab_v, conf_v = uk[m], cen[m], lab_v[m], conf_v[m]
    if a.labelled_only:
        m = lab_v != UNLABELED
        uk, cen, lab_v, conf_v = uk[m], cen[m], lab_v[m], conf_v[m]
    rgb = palette[lab_v]
    write_labeled(out / f"{a.name}.pcd", cen.astype(np.float32), rgb, lab_v, conf_v)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cen))
    pc.colors = o3d.utility.Vector3dVector(rgb.astype(np.float64) / 255.0)
    o3d.io.write_point_cloud(str(out / f"{a.name}.ply"), pc, write_ascii=False)

    labelled = (lab_v != UNLABELED).sum()
    ids, counts = np.unique(lab_v[lab_v != UNLABELED], return_counts=True)
    per_class = sorted(
        ((int(c), id2label[int(i)], int(i)) for i, c in zip(ids, counts)), reverse=True
    )
    stats = {
        "voxel_m": a.voxel,
        "voxels": int(len(uk)),
        "labelled_voxels": int(labelled),
        "labelled_fraction": float(labelled / len(uk)),
        "points": int(n_pts),
        "labelled_observations": int(len(keys)),
        "mean_label_purity": float(conf_v[lab_v != UNLABELED].mean()),
        "classes": [{"id": i, "name": n, "voxels": c} for c, n, i in per_class],
    }
    json.dump(stats, open(out / f"{a.name}_stats.json", "w"), indent=1)
    print(
        f"voxels {len(uk)}, labelled {labelled} ({labelled / len(uk):.0%}), mean purity {stats['mean_label_purity']:.2f}"
    )
    for c, n, i in per_class[:25]:
        print(f"  {n:16s} {c:8d}  {c / labelled:5.1%}")
    print("wrote", out / f"{a.name}.pcd", out / f"{a.name}.ply", out / f"{a.name}_stats.json")


if __name__ == "__main__":
    main()
