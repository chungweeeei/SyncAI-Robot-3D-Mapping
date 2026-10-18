"""Instance-level object map (stage 3 of the object notes): 2D instance masks per keyframe,
projected onto the keyframe's patch, associated across keyframes in the loop-corrected world
frame, then the same per-object record as objects.py (class, colour, crops, SigLIP 2 embedding,
goal pose), so query_objects.py reads either.

    instances.py <map dir> <bag dir> <keyframes.txt> <lio_odom.txt> <out dir> [--backend sam3]

Why masks and not clustering: objects.py clusters labelled voxels by distance, which cannot tell
two touching sofas apart and splits one sofa seen in halves (42 sofa / armchair instances on
dp1f_with_image for a few lounges). A mask says "these pixels are one thing" in the frame it was
seen in; the 3D association only has to decide whether two such things are the same.

Association: each mask's points become a set of 0.2 m world voxel keys; a segment joins the
existing object of the same coarse group it overlaps most (shared keys / segment keys >=
--match), else starts a new one; afterwards objects of a group that overlap by --merge of the
smaller are merged until nothing changes. An object needs --min-obs observations, which is what
removes one-frame false detections.
"""

from __future__ import annotations

import argparse
import json
import pickle
import time
import zlib
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

from bagio import ImageIndex, body_to_camera, project, read_static
from instance_seg import make_instance_segmenter
from keyframes import load_poses_txt
from label_patches import OdomInterp, load_keyframes
from objects import LIDAR_HEIGHT, Embedder, color_name, goal_pose, write_gallery, write_topdown
from pcdio import read_xyz
from taxonomy import COARSE_ID, coarse_of_name

KEY = 0.2  # association voxel, m
FINE = 0.1  # geometry voxel, m


def keys(p: np.ndarray, v: float) -> np.ndarray:
    k = np.floor(p / v).astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def unkey(k: np.ndarray, v: float) -> np.ndarray:
    m = (1 << 21) - 1
    return (np.stack([(k >> 42) & m, (k >> 21) & m, k & m], 1) - (1 << 20) + 0.5) * v


def group_of(label: str) -> int:
    g = coarse_of_name(label)
    # Grounding DINO / SAM 3 prompts that the ADE taxonomy has no group for stay apart by name
    return g if g != COARSE_ID["other"] else 100 + zlib.crc32(label.encode()) % 1000


def segment_frames(a, map_dir, names, kt, kq, kfs, cam, T_cam_body, odom, idx, kf_tree):
    """Phase 1, the slow one: instance masks per image, each mask's visible patch points in the
    world frame, and its view score / crop. Cached to segments_<backend>.pkl."""
    W, H = cam.width, cam.height
    seg = make_instance_segmenter(a.backend)
    kernel = np.ones((2 * a.erode + 1, 2 * a.erode + 1), np.uint8) if a.erode else None
    sel = list(range(0, len(kfs), a.stride))
    if a.limit:
        sel = sel[: a.limit]
    by_image = {}
    for k in sel:
        j, dt = idx.nearest(kfs[k][1])
        if dt < 60e6:
            by_image.setdefault(j, k)  # one keyframe per image
    print(f"{len(sel)} keyframes -> {len(by_image)} images, backend {a.backend}")
    segments, n_masks, seg_s, t0 = [], 0, 0.0, time.time()
    for done, (j, stamp, img) in enumerate(idx.read_many(sorted(by_image))):
        k = by_image[j]
        ts = time.time()
        inst = seg(img)
        seg_s += time.time() - ts
        n_masks += len(inst)
        if not inst:
            continue
        # patch -> world (corrected pose) and -> camera at the image stamp (odometry)
        body = read_xyz(map_dir / "patches" / names[k]).astype(np.float64)
        world = R.from_quat(kq[k]).apply(body) + kt[k]
        R_kf, t_kf = odom.pose(kfs[k][1])
        R_im, t_im = odom.pose(stamp)
        pc = T_cam_body.apply(R_im.inv().apply(R_kf.apply(body) + t_kf - t_im))
        z = pc[:, 2]
        cone = (
            (z > 0.3)
            & (z < a.max_range)
            & (np.abs(pc[:, 0] / np.maximum(z, 1e-6)) < 1.1)
            & (np.abs(pc[:, 1] / np.maximum(z, 1e-6)) < 0.8)
        )
        ci = np.nonzero(cone)[0]
        uv = project(cam, pc[ci])
        ins = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
        ci, uv, zc = ci[ins], uv[ins], z[ci][ins]
        # occlusion: nearest point per 8 px cell (as in label_patches)
        cell = (uv[:, 1].astype(int) // 8) * (W // 8 + 1) + uv[:, 0].astype(int) // 8
        order = np.lexsort((zc, cell))
        cs, zs = cell[order], zc[order]
        first = np.r_[True, cs[1:] != cs[:-1]]
        zmin = np.repeat(zs[first], np.diff(np.r_[np.nonzero(first)[0], len(cs)]))
        vis = order[zs <= zmin + np.maximum(0.3, 0.1 * zmin)]
        ci, uv, zc = ci[vis], uv[vis], zc[vis]
        ui, vi = uv[:, 0].astype(int), uv[:, 1].astype(int)
        # floor under each point: nearest keyframe z - lidar height; mask points on the floor
        # are the mask's edge bleeding onto it, never the object
        _, nn = kf_tree.query(world[ci, :2])
        h = world[ci, 2] - (kt[nn, 2] - LIDAR_HEIGHT)
        for mask, label, score in inst:
            m = cv2.erode(mask.astype(np.uint8), kernel) > 0 if kernel is not None else mask
            hit = m[vi, ui] & (h > 0.05)
            if hit.sum() < 8:
                continue
            ys, xs = np.nonzero(mask)
            x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
            d = float(np.median(zc[hit]))
            cut = (x0 < 3) + (y0 < 3) + (x1 > W - 4) + (y1 > H - 4)
            area = (x1 - x0) * (y1 - y0) / (W * H)
            vs = score * min(1.0, area / 0.04) * np.exp(-(((d - 3.0) / 2.5) ** 2)) * (0.3**cut)
            cx, cy, half = (x0 + x1) / 2, (y0 + y1) / 2, max(x1 - x0, y1 - y0, 96) * 0.6
            crop = img[
                int(max(0, cy - half)) : int(min(H, cy + half)),
                int(max(0, cx - half)) : int(min(W, cx + half)),
            ]
            rgb = img[m][:, ::-1] if m.any() else img[mask][:, ::-1]
            segments.append(
                {
                    "k": k,
                    "j": j,
                    "label": label,
                    "score": float(score),
                    "p": world[ci[hit]].astype(np.float32),
                    "d": d,
                    "vs": float(vs),
                    "crop": cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes(),
                    "rgb": rgb[:: max(1, len(rgb) // 4000)].copy(),
                }
            )
        if done % 50 == 0:
            print(
                f"  {done}/{len(by_image)} images, {n_masks} masks, {len(segments)} segments, {seg_s / max(done, 1):.2f} s/img seg, {time.time() - t0:.0f} s",
                flush=True,
            )
    run = {
        "images": len(by_image),
        "masks": n_masks,
        "seconds_segmentation": round(seg_s, 1),
        "seconds_per_image_seg": round(seg_s / max(len(by_image), 1), 3),
        "prompts": seg.prompts,
    }
    return segments, run, seg.prompts


class Obj:
    def __init__(self, group):
        self.group = group
        self.pts = np.zeros((0, 3), np.float32)
        self.f: dict[int, int] = defaultdict(int)  # 0.1 m key -> observations
        self.labels: dict[str, float] = defaultdict(float)
        self.obs = 0
        self.views: list = []  # (score, kf, image idx, dist, crop jpg, rgb sample)
        self._tree = None

    def add(self, s):
        self.pts = np.concatenate([self.pts, s["p"]])
        for key in np.unique(keys(s["p"], FINE)):
            self.f[int(key)] += 1
        self.labels[s["label"]] += s["score"]
        self.obs += 1
        self.views = sorted(
            self.views + [(s["vs"], s["k"], s["j"], s["d"], s["crop"], s["rgb"])],
            key=lambda v: -v[0],
        )[:4]
        self._tree = None

    def absorb(self, o: "Obj"):
        self.pts = np.concatenate([self.pts, o.pts])
        for key, n in o.f.items():
            self.f[key] += n
        for lab, w in o.labels.items():
            self.labels[lab] += w
        self.obs += o.obs
        self.views = sorted(self.views + o.views, key=lambda v: -v[0])[:4]
        self._tree = None

    @property
    def tree(self):
        # In the floor plane: views from different sides hit different faces of a piece of
        # furniture (front, seat, back), far apart in 3D but on one footprint seen from above.
        if self._tree is None:
            self._tree = cKDTree(self.pts[:, :2])
        return self._tree

    @property
    def centroid(self):
        return self.pts.mean(0)

    def box(self, margin=0.0):
        return self.pts[:, :2].min(0) - margin, self.pts[:, :2].max(0) + margin

    def near_share(self, p, near):
        return float(np.isfinite(self.tree.query(p[:, :2], distance_upper_bound=near)[0]).mean())


def diag(lo, hi):
    return float(np.linalg.norm(hi - lo))


def clean_segment(p: np.ndarray, eps: float) -> np.ndarray:
    """The largest 3D cluster of a mask's points. A mask's ~20 LiDAR points are mostly the
    object, but a few go through gaps (between sofa legs, past a chair back) or off the mask's
    edge onto whatever is behind: on dp1f_with_image a sofa segment's robust footprint was 1.4 m
    and its full one 2.4 m (p90 4.4 m), which chained objects together in the association."""
    if len(p) < 6:
        return p
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p.astype(np.float64)))
    lab = np.asarray(pc.cluster_dbscan(eps=eps, min_points=3))
    if lab.max() < 0:
        return p[:0]
    best = np.bincount(lab[lab >= 0]).argmax()
    return p[lab == best]


def associate(segments, a):
    """Phase 2: segments -> objects. The LiDAR points a mask gets are sparse and from the
    surface facing that keyframe, so two views of one sofa rarely share voxels; what they share
    is proximity (points within --near of each other) and footprint (one view's centroid inside
    the box the others built), both bounded by --max-extent."""
    objs: list[Obj] = []
    n_clean = 0
    for s in segments:
        s = dict(s, p=clean_segment(s["p"], a.segment_eps))
        if len(s["p"]) < 6:
            continue
        n_clean += 1
        g = group_of(s["label"])
        c = s["p"].mean(0)
        best, best_score = None, 0.0
        for o in objs:
            if o.group != g or np.linalg.norm(o.centroid[:2] - c[:2]) > 4.0:
                continue
            share = o.near_share(s["p"], a.near)
            lo, hi = o.box(a.bbox_margin)
            inside = bool(np.all(c[:2] >= lo) and np.all(c[:2] <= hi))
            # never grow an object past --max-extent: the footprint rule chained neighbouring
            # sofas into 5-7 m "objects" when it was unbounded
            both = np.concatenate([o.pts[:, :2], s["p"][:, :2]])
            if diag(both.min(0), both.max(0)) > a.max_extent:
                continue
            score = share + (0.5 if inside else 0.0)
            if (share >= a.match or inside) and score > best_score:
                best, best_score = o, score
        if best is None:
            best = Obj(g)
            objs.append(best)
        best.add(s)
    # merge objects of a group that are the same thing seen apart
    merged, rounds = True, 0
    while merged:
        merged, rounds = False, rounds + 1
        objs.sort(key=lambda o: -len(o.pts))
        alive = [True] * len(objs)
        for i, o in enumerate(objs):
            if not alive[i]:
                continue
            for j in range(i + 1, len(objs)):
                q = objs[j]
                if (
                    not alive[j]
                    or q.group != o.group
                    or np.linalg.norm(o.centroid[:2] - q.centroid[:2]) > 4.0
                ):
                    continue
                both = np.concatenate([o.pts[:, :2], q.pts[:, :2]])
                if diag(both.min(0), both.max(0)) > a.max_extent:
                    continue
                if o.near_share(q.pts, a.near) >= a.merge:
                    o.absorb(q)
                    alive[j] = False
                    merged = True
        objs = [o for o, keep in zip(objs, alive) if keep]
    print(
        f"association: {len(segments)} segments ({n_clean} after cleaning) -> {len(objs)} objects after {rounds} merge rounds"
    )
    return objs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_dir")
    ap.add_argument("bag")
    ap.add_argument("keyframes")
    ap.add_argument("odom")
    ap.add_argument("out")
    ap.add_argument("--backend", default="grounded-sam", help="sam3 | grounded-sam")
    ap.add_argument("--stride", type=int, default=2, help="segment every N-th keyframe")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--erode", type=int, default=5, help="mask erosion, px, against edge bleed")
    ap.add_argument(
        "--match",
        type=float,
        default=0.35,
        help="share of a segment's points near an object to join it",
    )
    ap.add_argument(
        "--merge",
        type=float,
        default=0.35,
        help="share of the smaller object's points near the larger to merge",
    )
    ap.add_argument(
        "--near", type=float, default=0.3, help="'near', m: xy point-to-point distance for both"
    )
    ap.add_argument(
        "--bbox-margin",
        type=float,
        default=0.15,
        help="segment centroid inside an object's grown xy box also joins it",
    )
    ap.add_argument(
        "--reuse",
        action="store_true",
        help="reuse the cached per-frame segments, re-run only the association",
    )
    ap.add_argument("--min-obs", type=int, default=2)
    ap.add_argument("--min-voxels", type=int, default=6, help="a fire extinguisher is ~10 voxels")
    ap.add_argument(
        "--segment-eps", type=float, default=0.4, help="DBSCAN radius cleaning a mask's points, m"
    )
    ap.add_argument("--max-extent", type=float, default=3.5, help="largest object xy diagonal, m")
    ap.add_argument("--max-range", type=float, default=8.0, help="ignore mask points farther, m")
    ap.add_argument("--model", default="google/siglip2-so400m-patch14-384")
    a = ap.parse_args()
    map_dir, out = Path(a.map_dir), Path(a.out)
    (out / "crops").mkdir(parents=True, exist_ok=True)
    t0 = time.time()

    names, kt, kq = load_poses_txt(map_dir / "poses.txt")
    kfs = load_keyframes(Path(a.keyframes))
    assert [k[0] for k in kfs] == names, "keyframes.txt does not match poses.txt"
    cam, tfs = read_static(Path(a.bag))
    T_cam_body = body_to_camera(tfs)
    odom = OdomInterp(Path(a.odom))
    idx = ImageIndex(Path(a.bag))
    kf_tree = cKDTree(kt[:, :2])
    cache = out / f"segments_{a.backend}.pkl"
    if a.reuse and cache.exists():
        segments = pickle.load(open(cache, "rb"))
        meta_run = segments.pop()  # last entry: the run record
        prompts = meta_run["prompts"]
        print(f"reusing {len(segments)} segments from {cache.name}")
    else:
        segments, meta_run, prompts = segment_frames(
            a, map_dir, names, kt, kq, kfs, cam, T_cam_body, odom, idx, kf_tree
        )
        pickle.dump(segments + [meta_run], open(cache, "wb"))
    n_masks = meta_run["masks"]
    objs = associate(segments, a)
    keep = []
    for o in objs:
        if o.obs < a.min_obs:
            continue
        # geometry: fine voxels seen in a fifth of the observations (one-frame bleed drops out)
        f = np.array([key for key, n in o.f.items() if n >= max(1, 0.2 * o.obs)], np.int64)
        if len(f) < a.min_voxels or not o.views:
            continue
        keep.append((o, unkey(f, FINE)))
    print(
        f"{n_masks} masks, {len(segments)} segments -> {len(objs)} objects -> {len(keep)} kept (>= {a.min_obs} obs, >= {a.min_voxels} voxels) ({time.time() - t0:.0f} s)"
    )

    # floor / obstacles for the goal poses: from the saved map itself (no semantic run needed)
    mp = read_xyz(map_dir / "map.pcd").astype(np.float64)
    mp = unkey(np.unique(keys(mp, FINE)), FINE)
    _, nn = kf_tree.query(mp[:, :2])
    hm = mp[:, 2] - (kt[nn, 2] - LIDAR_HEIGHT)
    floor = np.abs(hm) < 0.1
    obst = (hm > 0.15) & (hm < 1.5)
    floor_tree, obst_tree, floor_xyz = cKDTree(mp[floor, :2]), cKDTree(mp[obst, :2]), mp[floor]

    emb_model = Embedder(a.model)
    flat = [
        (i, cv2.imdecode(np.frombuffer(v[4], np.uint8), cv2.IMREAD_COLOR))
        for i, (o, _) in enumerate(keep)
        for v in o.views[:3]
    ]
    E = emb_model.images([c for _, c in flat])
    emb = np.zeros((len(keep), E.shape[1]), np.float32)
    for (i, _), e in zip(flat, E):
        emb[i] += e
    records, rows = [], []
    for i, (o, pts) in enumerate(keep):
        label = max(o.labels, key=o.labels.get)
        oid = len(records)
        views = []
        for n, (vs, k, j, d, crop, _) in enumerate(o.views[:3]):
            fn = f"crops/{oid:04d}_{n}.jpg"
            (out / fn).write_bytes(crop)
            views.append(
                {
                    "file": fn,
                    "keyframe": names[k],
                    "image_index": int(j),
                    "distance_m": round(d, 2),
                    "score": round(float(vs), 3),
                }
            )
        rgb = np.concatenate([v[5] for v in o.views[:3]])
        cen = pts.mean(0)
        records.append(
            {
                "id": oid,
                "class": label,
                "group": int(o.group),
                "class_votes": {
                    k: round(v, 2) for k, v in sorted(o.labels.items(), key=lambda t: -t[1])[:4]
                },
                "observations": int(o.obs),
                "voxels": int(len(pts)),
                "centroid": [round(float(v), 3) for v in cen],
                "bbox_min": [round(float(v), 3) for v in pts.min(0)],
                "bbox_max": [round(float(v), 3) for v in pts.max(0)],
                "color": color_name(rgb),
                "color_rgb_median": [int(v) for v in np.median(rgb, 0)],
                "views": views,
                "goal": goal_pose(pts, floor_tree, floor_xyz, obst_tree),
            }
        )
        rows.append(emb[i] / max(np.linalg.norm(emb[i]), 1e-9))
    np.save(out / "objects_emb.npy", np.array(rows, np.float16))
    json.dump(
        {
            "frame_id": "map",
            "source": {
                "instance_backend": a.backend,
                "embedding_model": a.model,
                "prompts": prompts,
            },
            "params": {
                k: getattr(a, k)
                for k in (
                    "stride",
                    "erode",
                    "match",
                    "merge",
                    "near",
                    "bbox_margin",
                    "min_obs",
                    "min_voxels",
                    "max_range",
                )
            },
            "run": {
                **{k: v for k, v in meta_run.items() if k != "prompts"},
                "segments": len(segments),
                "seconds_total": round(time.time() - t0, 1),
            },
            "objects": records,
        },
        open(out / "objects.json", "w"),
        indent=1,
        ensure_ascii=False,
    )
    write_gallery(out, records)
    coarse = np.where(floor, COARSE_ID["floor"], COARSE_ID["wall"]).astype(np.uint8)
    write_topdown(out, mp, coarse, records)
    by = defaultdict(int)
    for r in records:
        by[r["class"]] += 1
    print(
        f"{len(records)} objects in {time.time() - t0:.0f} s: "
        + ", ".join(f"{c} {n}" for c, n in sorted(by.items(), key=lambda t: -t[1]))
    )
    print("wrote", out / "objects.json")


if __name__ == "__main__":
    main()
