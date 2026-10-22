"""Object-level map from a fused semantic map (stages 1 + 2 of the object notes in
docs/semantic-mapping.md): cluster the "thing" voxels into object instances, go back to the
camera images for each one's best views, and store per object a class, a colour, a few crops and
one image embedding (SigLIP 2), so a text query ("green sofa", "綠色沙發") can be answered after
mapping without a per-pixel feature map.

    objects.py <map dir> <run dir> <bag dir> <keyframes.txt> <lio_odom.txt> <out dir>

<run dir> is a label_patches.py + fuse.py run (its map_semantic_clean.pcd, classes.json and
label_stats.txt). Writes <out dir>/objects.json, objects_emb.npy (float16, one row per object in
objects.json order), crops/<id>_<n>.jpg and index.html. Nothing in objects.json names the map or
holds an absolute path: crops are relative to the file, keyframes are patch basenames.

Why per-object crops and not CLIP features per voxel: a query only ever needs to rank objects, and
an object's best two or three views carry what the text can ask about (class, colour, material,
what it stands next to). That is a few hundred image encodes for a whole map instead of a dense
feature map per keyframe, and the voxels stay a plain class histogram.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

from bagio import ImageIndex, body_to_camera, project, read_static, view_cone
from keyframes import load_poses_txt
from label_patches import OdomInterp, load_keyframes
from pcdio import read_labeled, read_xyz
from taxonomy import COARSE, COARSE_ID, coarse_lut

# Coarse classes that are surfaces or passers-by, not objects to find: never clustered.
NOT_OBJECTS = {"floor", "wall", "ceiling", "window", "column", "person", "other_stuff"}
# Fine ADE classes folded into "other" that are still stuff (planes, not things).
FINE_STUFF = {
    "building", "sky", "earth", "grass", "mountain", "water", "sea", "sand", "field", "rock",
    "hill", "river", "dirt track", "land", "skyscraper", "house", "railing", "bannister", "base",
    "blind", "rug", "sidewalk", "road", "path", "stage", "awning", "curtain", "mirror", "painting",
}  # fmt: skip

LIDAR_HEIGHT = 0.481  # pgo's map_clean lidar height: floor = keyframe z - this


def color_name(rgb: np.ndarray) -> str:
    """A coarse colour word for an (N, 3) uint8 RGB sample: medians in HSV, hue from the
    saturated pixels only. Deliberately coarse: the camera's white balance is latched, so a
    finer name would describe the lighting more than the object."""
    hsv = cv2.cvtColor(rgb.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)
    h, s, v = hsv[:, 0].astype(float) * 2, hsv[:, 1].astype(float), hsv[:, 2].astype(float)
    sm, vm = np.median(s), np.median(v)
    if vm < 50:
        return "black"
    if sm < 45:
        return "white" if vm > 185 else "gray"
    sat = s >= np.percentile(s, 50)
    ang = np.deg2rad(h[sat])
    hue = np.rad2deg(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) % 360
    if hue < 15 or hue >= 340:
        name = "red"
    elif hue < 40:
        name = "orange"
    elif hue < 65:
        name = "yellow"
    elif hue < 170:
        name = "green"
    elif hue < 200:
        name = "cyan"
    elif hue < 255:
        name = "blue"
    elif hue < 290:
        name = "purple"
    else:
        name = "pink"
    if name in ("red", "orange", "yellow") and vm < 150:
        return "brown"
    return name


class Embedder:
    """SigLIP 2 image / text tower; embeddings L2-normalised, so a dot product is the cosine."""

    def __init__(self, name="google/siglip2-so400m-patch14-384", device=None):
        import torch
        from transformers import AutoModel, AutoProcessor

        self.torch = torch
        self.device = device or ("mps" if torch.backends.mps.is_available() else "cpu")
        self.name = name
        self.proc = AutoProcessor.from_pretrained(name)
        self.model = AutoModel.from_pretrained(name).float().to(self.device).eval()

    def _norm(self, x):
        if not isinstance(x, self.torch.Tensor):  # newer transformers return an output object
            x = x.pooler_output
        x = x / x.norm(dim=-1, keepdim=True)
        return x.float().cpu().numpy()

    def images(self, bgr_list):
        from PIL import Image

        out = []
        with self.torch.no_grad():
            for i in range(0, len(bgr_list), 16):
                pil = [
                    Image.fromarray(np.ascontiguousarray(b[:, :, ::-1]))
                    for b in bgr_list[i : i + 16]
                ]
                inp = self.proc(images=pil, return_tensors="pt").to(self.device)
                out.append(self._norm(self.model.get_image_features(**inp)))
        return np.concatenate(out) if out else np.zeros((0, 1), np.float32)

    def texts(self, texts):
        with self.torch.no_grad():
            inp = self.proc(
                text=texts,
                padding="max_length",
                max_length=64,
                truncation=True,
                return_tensors="pt",
            ).to(self.device)
            return self._norm(self.model.get_text_features(**inp))

    def probability(self, cos: np.ndarray) -> np.ndarray:
        """SigLIP's own calibrated match probability for a cosine."""
        scale = float(self.model.logit_scale.exp())
        bias = float(self.model.logit_bias)
        return 1 / (1 + np.exp(-(cos * scale + bias)))


def cluster_objects(xyz, lab, conf, id2label, eps, min_voxels, min_conf):
    """Instances per coarse group (sofa and armchair labels on one object stay one object);
    the instance's class is its majority fine label."""
    lut = coarse_lut(id2label)
    coarse = lut[lab]
    objects = []
    for cid, cname in enumerate(COARSE):
        if cname in NOT_OBJECTS:
            continue
        m = (coarse == cid) & (conf >= min_conf)
        if cname == "other":  # keep things from "other", drop the stuff folded into it
            m &= np.array([id2label.get(int(i), "").split(",")[0] not in FINE_STUFF for i in lab])
        idx = np.nonzero(m)[0]
        if len(idx) < min_voxels:
            continue
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[idx].astype(np.float64)))
        cl = np.asarray(pc.cluster_dbscan(eps=eps, min_points=4))
        for c in range(cl.max() + 1 if len(cl) else 0):
            sel = idx[cl == c]
            if len(sel) < min_voxels:
                continue
            ids, cnt = np.unique(lab[sel], return_counts=True)
            fine = int(ids[np.argmax(cnt)])
            objects.append(
                {
                    "group": cname,
                    "class": id2label[fine].split(",")[0].strip(),
                    "class_votes": {
                        id2label[int(i)].split(",")[0].strip(): int(n)
                        for i, n in sorted(zip(ids, cnt), key=lambda t: -t[1])[:4]
                    },
                    "voxel_idx": sel,
                }
            )
    return objects


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_dir")
    ap.add_argument("run_dir")
    ap.add_argument("bag")
    ap.add_argument("keyframes")
    ap.add_argument("odom")
    ap.add_argument("out")
    ap.add_argument("--name", default="map_semantic_clean")
    ap.add_argument("--eps", type=float, default=0.22, help="DBSCAN radius, m (0.1 m voxels)")
    ap.add_argument("--min-voxels", type=int, default=25)
    ap.add_argument("--min-conf", type=float, default=0.5, help="voxel vote purity to cluster it")
    ap.add_argument("--views", type=int, default=3)
    ap.add_argument(
        "--view-radius", type=float, default=7.0, help="keyframes this close are candidates"
    )
    ap.add_argument("--model", default="google/siglip2-so400m-patch14-384")
    a = ap.parse_args()
    map_dir, run, out = Path(a.map_dir), Path(a.run_dir), Path(a.out)
    (out / "crops").mkdir(parents=True, exist_ok=True)
    t0 = time.time()

    classes = json.load(open(run / "classes.json"))
    id2label = {int(k): v for k, v in classes["id2label"].items()}
    xyz, lab, conf = read_labeled(run / f"{a.name}.pcd")
    lab = lab.astype(np.int64)
    objs = cluster_objects(xyz, lab, conf, id2label, a.eps, a.min_voxels, a.min_conf)
    print(f"{len(objs)} object instances from {len(xyz)} voxels ({time.time() - t0:.0f} s)")

    # keyframes: corrected world pose (poses.txt), odom pose + stamp (keyframes.txt), image (label_stats)
    names, kt, kq = load_poses_txt(map_dir / "poses.txt")
    kfs = load_keyframes(Path(a.keyframes))
    assert [k[0] for k in kfs] == names, "keyframes.txt does not match poses.txt"
    img_of = {
        int(r.split()[0]): int(r.split()[1])
        for r in (run / "label_stats.txt").read_text().splitlines()
    }
    cam, tfs = read_static(Path(a.bag))
    T_cam_body = body_to_camera(tfs)
    odom = OdomInterp(Path(a.odom))
    idx = ImageIndex(Path(a.bag))
    kf_tree = cKDTree(kt[:, :2])
    W, H = cam.width, cam.height

    # floor height under every point: nearest keyframe z minus the lidar height
    _, nn = kf_tree.query(xyz[:, :2])
    h_all = xyz[:, 2] - (kt[nn, 2] - LIDAR_HEIGHT)
    coarse_all = coarse_lut(id2label)[lab]

    patch_cache: dict[int, np.ndarray] = {}

    def to_cam(world: np.ndarray, k: int):
        """World points -> camera frame at keyframe k's image stamp."""
        Rw = R.from_quat(kq[k])
        p_body_kf = Rw.inv().apply(world - kt[k])
        R_kf, t_kf = odom.pose(kfs[k][1])
        R_im, t_im = odom.pose(idx.stamps[img_of[k]])
        p_body_im = R_im.inv().apply(R_kf.apply(p_body_kf) + t_kf - t_im)
        return T_cam_body.apply(p_body_im)

    def zbuffer(k: int, cell=8):
        if k not in patch_cache:
            if len(patch_cache) > 64:
                patch_cache.pop(next(iter(patch_cache)))
            body = read_xyz(map_dir / "patches" / names[k]).astype(np.float64)
            world = R.from_quat(kq[k]).apply(body) + kt[k]
            pc = to_cam(world, k)
            ok = pc[:, 2] > 0.2
            pc = pc[ok]
            uv = project(cam, pc)
            ins = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
            zb = np.full((H // cell + 1, W // cell + 1), np.inf)
            np.minimum.at(
                zb, ((uv[ins, 1] // cell).astype(int), (uv[ins, 0] // cell).astype(int)), pc[ins, 2]
            )
            patch_cache[k] = zb
        return patch_cache[k]

    # --- pick views: visible voxels (frustum + z-buffer), distance, not cut by the image border
    views_by_image = defaultdict(list)
    for oi, o in enumerate(objs):
        pts = xyz[o["voxel_idx"]].astype(np.float64)
        cen = pts.mean(0)
        cands = [k for k in kf_tree.query_ball_point(cen[:2], a.view_radius) if k in img_of]
        scored = []
        for k in cands:
            pc = to_cam(pts, k)
            z = pc[:, 2]
            cone = view_cone(cam, pc) & (z > 0.4)
            if cone.sum() < max(10, 0.3 * len(pts)):
                continue
            uv = project(cam, pc[cone])
            ins = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
            uv, zc = uv[ins], z[cone][ins]
            zb = zbuffer(k)
            zmin = zb[(uv[:, 1] // 8).astype(int), (uv[:, 0] // 8).astype(int)]
            vis = zc <= zmin + np.maximum(0.3, 0.1 * zmin)
            if vis.sum() < max(10, 0.3 * len(pts)):
                continue
            uvv = uv[vis]
            x0, y0 = uvv.min(0)
            x1, y1 = uvv.max(0)
            cut = (x0 < 4) + (y0 < 4) + (x1 > W - 4) + (y1 > H - 4)
            d = float(np.median(zc[vis]))
            # A first run kept views from under a metre that filled the frame with one cushion
            # (or, with the extrinsic error, a wall beside it): a crop has to show the object
            # whole enough for its shape to say what it is.
            if d < 1.2 or (x1 - x0) * (y1 - y0) > 0.6 * W * H:
                continue
            score = (
                vis.mean()
                * min(1.0, vis.sum() / len(pts))
                * np.exp(-(((d - 3.0) / 2.5) ** 2))
                * (0.3**cut)
            )
            scored.append((score, k, uvv, d))
        scored.sort(key=lambda s: -s[0])
        chosen = []
        for s in scored:  # distinct viewpoints: at least 4 keyframes apart
            if all(abs(s[1] - c[1]) >= 4 for c in chosen):
                chosen.append(s)
            if len(chosen) == a.views:
                break
        o["views"] = []
        for score, k, uvv, d in chosen:
            views_by_image[img_of[k]].append((oi, k, uvv, float(score), d))
    n_seen = len({v[0] for vs in views_by_image.values() for v in vs})
    print(
        f"views chosen for {n_seen} of {len(objs)} objects, {len(views_by_image)} images to read ({time.time() - t0:.0f} s)"
    )

    # --- read the images once, in bag order: crops and colour samples
    crops = defaultdict(list)
    samples = defaultdict(list)
    for j, _, img in idx.read_many(sorted(views_by_image)):
        for oi, k, uvv, score, d in views_by_image[j]:
            x0, y0 = uvv.min(0)
            x1, y1 = uvv.max(0)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            half = max(x1 - x0, y1 - y0, 96) * 0.6
            xa, xb = int(max(0, cx - half)), int(min(W, cx + half))
            ya, yb = int(max(0, cy - half)), int(min(H, cy + half))
            crop = img[ya:yb, xa:xb].copy()
            # colour from the pixels the object's own voxels land on (3x3 each), not the box
            ui = np.clip(uvv[:, 0].astype(int), 1, W - 2)
            vi = np.clip(uvv[:, 1].astype(int), 1, H - 2)
            pix = np.concatenate([img[vi + dy, ui + dx] for dy in (-1, 0, 1) for dx in (-1, 0, 1)])
            samples[oi].append(pix[:, ::-1])  # RGB
            vis = crop.copy()
            for u, v in zip(ui - xa, vi - ya):
                cv2.circle(vis, (int(u), int(v)), 1, (0, 255, 255), -1)
            crops[oi].append((score, k, j, d, crop, vis))
    print(f"crops read ({time.time() - t0:.0f} s)")

    # --- embeddings: mean of the views' normalised SigLIP vectors, renormalised
    emb_model = Embedder(a.model)
    flat = [(oi, c[4]) for oi in sorted(crops) for c in crops[oi]]
    E = emb_model.images([c for _, c in flat])
    emb = np.zeros((len(objs), E.shape[1]), np.float32)
    for (oi, _), e in zip(flat, E):
        emb[oi] += e
    print(f"{len(flat)} crops embedded ({time.time() - t0:.0f} s)")

    # --- navigable goal: free floor voxel 0.6-1.5 m from the object, facing it
    floor = (coarse_all == COARSE_ID["floor"]) & (np.abs(h_all) < 0.2)
    obst = (h_all > 0.15) & (h_all < 1.5) & ~floor
    floor_tree = cKDTree(xyz[floor, :2])
    obst_tree = cKDTree(xyz[obst, :2])
    floor_xyz = xyz[floor]

    records, rows = [], []
    for oi, o in enumerate(objs):
        if oi not in crops:
            continue
        pts = xyz[o["voxel_idx"]]
        cen = pts.mean(0)
        e = emb[oi] / max(np.linalg.norm(emb[oi]), 1e-9)
        rgb = np.concatenate(samples[oi])
        goal = goal_pose(pts, floor_tree, floor_xyz, obst_tree)
        oid = len(records)
        crop_files = []
        for n, (score, k, j, d, crop, vis) in enumerate(sorted(crops[oi], key=lambda c: -c[0])):
            f = f"crops/{oid:04d}_{n}.jpg"
            cv2.imwrite(str(out / f), crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
            cv2.imwrite(
                str(out / f"crops/{oid:04d}_{n}_pts.jpg"), vis, [cv2.IMWRITE_JPEG_QUALITY, 80]
            )
            crop_files.append(
                {
                    "file": f,
                    "keyframe": names[k],
                    "image_index": int(j),
                    "distance_m": round(d, 2),
                    "score": round(score, 3),
                }
            )
        records.append(
            {
                "id": oid,
                "class": o["class"],
                "group": o["group"],
                "class_votes": o["class_votes"],
                "voxels": int(len(pts)),
                "centroid": [round(float(v), 3) for v in cen],
                "bbox_min": [round(float(v), 3) for v in pts.min(0)],
                "bbox_max": [round(float(v), 3) for v in pts.max(0)],
                "color": color_name(rgb),
                "color_rgb_median": [int(v) for v in np.median(rgb, 0)],
                "views": crop_files,
                "goal": goal,
            }
        )
        rows.append(e)
    np.save(out / "objects_emb.npy", np.array(rows, np.float16))
    json.dump(
        {
            "frame_id": "map",
            "source": {"semantic_run": run.name, "fused": a.name, "embedding_model": a.model},
            "params": {
                "eps": a.eps,
                "min_voxels": a.min_voxels,
                "min_conf": a.min_conf,
                "views": a.views,
            },
            "objects": records,
        },
        open(out / "objects.json", "w"),
        indent=1,
        ensure_ascii=False,
    )
    write_gallery(out, records)
    write_topdown(out, xyz, coarse_all, records)
    by = defaultdict(int)
    for r in records:
        by[r["class"]] += 1
    print(
        f"{len(records)} objects with views ({len(objs) - len(records)} never seen well) in {time.time() - t0:.0f} s"
    )
    print("  " + ", ".join(f"{c} {n}" for c, n in sorted(by.items(), key=lambda t: -t[1])[:25]))
    print("wrote", out / "objects.json", out / "index.html")


def goal_pose(pts, floor_tree, floor_xyz, obst_tree):
    """A free floor voxel 0.6-1.5 m from the object's nearest point (no obstacle voxel within the
    robot's 0.35 m radius), the one nearest the object, facing it; None if there is none."""
    cen = pts.mean(0)
    cand = floor_tree.query_ball_point(cen[:2], 2.0 + 0.5 * float(np.ptp(pts[:, :2], axis=0).max()))
    if not cand:
        return None
    fc = floor_xyz[cand]
    dobj = cKDTree(pts[:, :2]).query(fc[:, :2])[0]
    ok = (dobj > 0.6) & (dobj < 1.5)
    ok &= np.array([len(obst_tree.query_ball_point(q[:2], 0.35)) == 0 for q in fc])
    if not ok.any():
        return None
    best = fc[ok][np.argmin(np.linalg.norm(fc[ok, :2] - cen[:2], axis=1))]
    yaw = float(np.arctan2(cen[1] - best[1], cen[0] - best[0]))
    return {
        "frame_id": "map",
        "x": round(float(best[0]), 3),
        "y": round(float(best[1]), 3),
        "yaw": round(yaw, 3),
    }


def write_topdown(out: Path, xyz, coarse, records, res=0.1):
    """Floor / walls in grey, each object's bbox in its colour with its id: where things are."""
    mn = xyz[:, :2].min(0) - 1
    size = np.ceil((xyz[:, :2].max(0) + 1 - mn) / res).astype(int)
    img = np.full((size[1], size[0], 3), 20, np.uint8)
    ij = ((xyz[:, :2] - mn) / res).astype(int)
    floor = coarse == COARSE_ID["floor"]
    img[size[1] - 1 - ij[floor, 1], ij[floor, 0]] = (55, 55, 55)
    img[size[1] - 1 - ij[~floor, 1], ij[~floor, 0]] = (110, 110, 110)
    img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return int((x - mn[0]) / res * 2), int((size[1] - 1 - (y - mn[1]) / res) * 2)

    for r in records:
        c = tuple(int(v) for v in r["color_rgb_median"][::-1])
        a, b = px(r["bbox_min"][0], r["bbox_max"][1]), px(r["bbox_max"][0], r["bbox_min"][1])
        cv2.rectangle(img, a, b, (0, 220, 255), 2)
        cv2.putText(
            img,
            f"{r['id']} {r['class'][:6]}",
            (a[0], a[1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 220, 255),
            1,
        )
        cv2.circle(img, px(*r["centroid"][:2]), 4, c, -1)
        if r["goal"]:
            g = px(r["goal"]["x"], r["goal"]["y"])
            cv2.circle(img, g, 3, (0, 255, 0), -1)
    cv2.imwrite(str(out / "objects_topdown.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 85])


def write_gallery(out: Path, records):
    css = """body{font:13px system-ui;margin:16px;background:#111;color:#ddd}.g{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:10px}
.c{background:#1c1c1c;padding:6px;border-radius:4px}.c img{width:100%;height:160px;object-fit:contain;background:#000}
.sw{display:inline-block;width:12px;height:12px;vertical-align:middle;border:1px solid #555}small{color:#999}"""
    html = [
        f"<!doctype html><meta charset=utf-8><title>Object map</title><style>{css}</style>",
        f"<h1>{len(records)} objects</h1><div class=g>",
    ]
    for r in sorted(records, key=lambda r: (r["class"], -r["voxels"])):
        rgb = r["color_rgb_median"]
        img = r["views"][0]["file"] if r["views"] else ""
        g = r["goal"]
        html.append(
            f"<div class=c><img src='{img}' loading=lazy><div><b>#{r['id']} {r['class']}</b> "
            f"<span class=sw style='background:rgb({rgb[0]},{rgb[1]},{rgb[2]})'></span> {r['color']}</div>"
            f"<small>{r['voxels']} vox · ({r['centroid'][0]:.1f}, {r['centroid'][1]:.1f}) · "
            f"{len(r['views'])} views · goal {'yes' if g else 'none'}</small></div>"
        )
    html.append("</div>")
    (out / "index.html").write_text("\n".join(html))


if __name__ == "__main__":
    main()
