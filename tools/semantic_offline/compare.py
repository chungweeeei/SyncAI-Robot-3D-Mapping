"""Compare semantic-map runs made by different 2D backends on the same replay (same patches, same
projection, same fusion): fold every label space into the coarse taxonomy, score each run against
the map's own geometry (no annotation exists) and against a reference run, and write a table
(JSON + Markdown) and an HTML page with the renders and the QA overlays side by side.

    compare.py <map dir> <out dir> <run dir>...      (the first run dir is the reference)

Geometry as pseudo ground truth: the lidar sits `--lidar-height` above the floor, so a voxel's
height over the floor is z minus (nearest keyframe z - lidar height); normals come from the voxel
centroids. A "floor" voxel should be low and horizontal, a "ceiling" voxel high and horizontal,
a "wall" voxel vertical. These rates say nothing about doors or chairs, but they are the classes
traversability needs and they catch a model that paints the floor up the walls.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from keyframes import load_poses_txt
from pcdio import read_labeled
from taxonomy import COARSE, COARSE_ID, COARSE_PALETTE, UNLABELED, coarse_lut

F, W, C = COARSE_ID["floor"], COARSE_ID["wall"], COARSE_ID["ceiling"]


def voxel_key(xyz: np.ndarray, voxel: float) -> np.ndarray:
    k = np.floor(xyz / voxel).astype(np.int64)
    return (
        (k[:, 0] + (1 << 16)) * (1 << 34)
        + (k[:, 1] + (1 << 16)) * (1 << 17)
        + (k[:, 2] + (1 << 16))
    )


class Geometry:
    """Height over the local floor and |normal z| for every voxel of the reference geometry."""

    def __init__(self, xyz: np.ndarray, map_dir: Path, lidar_height: float, voxel: float):
        _, kt, _ = load_poses_txt(map_dir / "poses.txt")
        tree = cKDTree(kt[:, :2])
        _, nn = tree.query(xyz[:, :2], k=1)
        self.h = xyz[:, 2] - (kt[nn, 2] - lidar_height)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3 * voxel, max_nn=30))
        self.nz = np.abs(np.asarray(pc.normals)[:, 2])
        self.key = voxel_key(xyz, voxel)
        self.index = {int(k): i for i, k in enumerate(self.key)}
        # geometric floor / ceiling / wall, whatever the labels say
        self.geo_floor = (np.abs(self.h) < 0.15) & (self.nz > 0.8)
        self.geo_ceiling = (self.h > 1.8) & (self.nz > 0.8)
        self.geo_wall = (self.nz < 0.3) & (self.h > 0.1)

    def align(self, xyz: np.ndarray) -> np.ndarray:
        """Index into the reference geometry for every voxel of another run (-1 if absent)."""
        keys = voxel_key(xyz, 0.1)
        return np.array([self.index.get(int(k), -1) for k in keys], np.int64)


def coarse_topdown(xyz, coarse, path: Path, res=0.1):
    mn = xyz[:, :2].min(0)
    size = np.ceil((xyz[:, :2].max(0) - mn) / res).astype(int) + 1
    img = np.full((size[1], size[0], 3), 25, np.uint8)
    sel = np.nonzero(coarse != UNLABELED)[0]
    sel = sel[np.argsort(xyz[sel, 2])]
    ij = ((xyz[sel, :2] - mn) / res).astype(int)
    img[size[1] - 1 - ij[:, 1], ij[:, 0]] = COARSE_PALETTE[coarse[sel]][:, ::-1]
    cv2.imwrite(str(path), cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))


def score_run(run: Path, geo: Geometry, ref: dict | None, name: str) -> dict:
    classes = json.load(open(run / "classes.json"))
    lut = coarse_lut({int(k): v for k, v in classes["id2label"].items()})
    xyz, lab, conf = read_labeled(run / f"{name}.pcd")
    coarse = lut[lab]
    labelled = coarse != UNLABELED
    gi = geo.align(xyz)
    ok = gi >= 0
    h, nz = geo.h[gi], geo.nz[gi]
    geo_floor, geo_ceiling, geo_wall = (
        geo.geo_floor[gi] & ok,
        geo.geo_ceiling[gi] & ok,
        geo.geo_wall[gi] & ok,
    )

    def rate(mask, cond):
        n = int(mask.sum())
        return {"n": n, "rate": float((mask & cond).sum() / n) if n else None}

    is_f, is_w, is_c = coarse == F, coarse == W, coarse == C
    run_json = json.load(open(run / "run.json")) if (run / "run.json").exists() else {}
    stats = json.load(open(run / f"{name}_stats.json"))
    shares = {}
    for cid in range(len(COARSE)):
        n = int((coarse == cid).sum())
        if n:
            shares[COARSE[cid]] = n
    r = {
        "run": run.name,
        "model": classes.get("model", run.name),
        "seconds_per_image": run_json.get("seconds_per_image"),
        "device": run_json.get("device"),
        "stride": run_json.get("stride", 1),
        "keyframes": run_json.get("keyframes"),
        "voxels": int(len(xyz)),
        "labelled_fraction": float(labelled.mean()),
        "mean_purity": float(stats.get("mean_label_purity", float(conf[labelled].mean()))),
        "coarse_shares": shares,
        "fine_classes_used": int(len(np.unique(lab[labelled]))),
        # precision-like: of the voxels the model calls X, how many have X's geometry
        "floor_low_horizontal": rate(is_f & ok, (np.abs(h) < 0.25) & (nz > 0.7)),
        "wall_vertical": rate(is_w & ok, (nz < 0.4) & (h > 0.05)),
        "ceiling_high_horizontal": rate(is_c & ok, (h > 1.8) & (nz > 0.7)),
        # recall-like: of the labelled voxels with floor / ceiling / wall geometry, how many got the label
        "geo_floor_labelled_floor": rate(geo_floor & labelled, is_f),
        "geo_ceiling_labelled_ceiling": rate(geo_ceiling & labelled, is_c),
        "geo_wall_labelled_wall": rate(geo_wall & labelled, is_w),
        # ghosts: person voxels (the raw fusion keeps them; the clean mask removes them)
        "person_voxels": int((coarse == COARSE_ID["person"]).sum()),
    }
    if ref is not None:
        rk = ref["keys"]
        keys = voxel_key(xyz, 0.1)
        idx = np.array([rk.get(int(k), -1) for k in keys], np.int64)
        both = (idx >= 0) & labelled
        both &= ref["coarse"][np.maximum(idx, 0)] != UNLABELED
        rc = ref["coarse"][idx[both]]
        mc = coarse[both]
        r["agreement_with_reference"] = float((rc == mc).mean()) if both.any() else None
        r["agreement_n"] = int(both.sum())
        conf_m = {}
        for cid in (F, W, C):
            m = rc == cid
            if m.any():
                ids, cnt = np.unique(mc[m], return_counts=True)
                conf_m[COARSE[cid]] = {
                    COARSE[i]: round(float(c / m.sum()), 3)
                    for i, c in sorted(zip(ids, cnt), key=lambda t: -t[1])[:4]
                }
        r["reference_confusion"] = conf_m
    coarse_topdown(
        xyz, coarse, run / f"coarse_topdown{'' if name == 'map_semantic' else '_clean'}.jpg"
    )
    return r, {"keys": {int(k): i for i, k in enumerate(voxel_key(xyz, 0.1))}, "coarse": coarse}


def pct(x):
    return "–" if x is None else f"{100 * x:.0f} %"


def rate_cell(d):
    return "–" if d is None or d["rate"] is None else f"{100 * d['rate']:.0f} % ({d['n']:,})"


def write_markdown(rows: list[dict], out: Path):
    lines = [
        "| run | model | s / image | kf | labelled | purity | floor low+flat | wall vertical | ceiling high+flat | geo-floor → floor | geo-wall → wall | geo-ceiling → ceiling | agree w/ ref | person vox |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        spi = "–" if r["seconds_per_image"] is None else f"{r['seconds_per_image']:.2f}"
        kf = f"{r['keyframes']}" + (f" (stride {r['stride']})" if r.get("stride", 1) > 1 else "")
        lines.append(
            f"| {r['run']} | `{r['model']}` | {spi} | {kf} | {pct(r['labelled_fraction'])} | {r['mean_purity']:.2f} | "
            f"{rate_cell(r['floor_low_horizontal'])} | {rate_cell(r['wall_vertical'])} | {rate_cell(r['ceiling_high_horizontal'])} | "
            f"{rate_cell(r['geo_floor_labelled_floor'])} | {rate_cell(r['geo_wall_labelled_wall'])} | {rate_cell(r['geo_ceiling_labelled_ceiling'])} | "
            f"{pct(r.get('agreement_with_reference'))} | {r['person_voxels']} |"
        )
    lines += [
        "",
        "Coarse class shares (voxels):",
        "",
        "| run | " + " | ".join(COARSE) + " |",
        "|---|" + "---|" * len(COARSE),
    ]
    for r in rows:
        tot = sum(r["coarse_shares"].values()) or 1
        lines.append(
            f"| {r['run']} | "
            + " | ".join(f"{100 * r['coarse_shares'].get(c, 0) / tot:.1f}" for c in COARSE)
            + " |"
        )
    (out / "compare.md").write_text("\n".join(lines) + "\n")


def write_html(rows: list[dict], runs: list[Path], out: Path, name: str):
    rel = lambda p: str(Path(p).resolve().relative_to(out.resolve(), walk_up=True))  # noqa: E731
    qa_sets = [set(p.name for p in (r / "qa").glob("kf_*.jpg")) for r in runs]
    common = sorted(set.intersection(*qa_sets)) if qa_sets else []
    common = [c for c in common if int(re.findall(r"\d+", c)[0]) % 300 == 0] or common[:6]
    suffix = "" if name == "map_semantic" else "_clean"
    head = """<!doctype html><meta charset="utf-8"><title>Semantic map: 2D backend comparison</title>
<style>body{font:14px system-ui;margin:20px;background:#111;color:#eee}table{border-collapse:collapse}
td,th{border:1px solid #444;padding:4px 8px;text-align:right;white-space:nowrap}th{background:#222}td:first-child,td:nth-child(2){text-align:left}
img{max-width:100%;height:auto;display:block}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(380px,1fr));gap:12px}
h2{margin-top:36px}.cap{color:#aaa;font-size:12px}code{color:#9cf}</style>"""
    legend = "".join(
        f'<span style="display:inline-block;margin:2px 8px 2px 0"><span style="display:inline-block;width:14px;height:14px;background:rgb{tuple(int(v) for v in COARSE_PALETTE[i])};vertical-align:middle"></span> {c}</span>'
        for i, c in enumerate(COARSE)
    )
    md = (out / "compare.md").read_text().splitlines()
    table = ["<table>"]
    for line in md:
        if not line.startswith("|") or set(line) <= set("|-"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        tag = "th" if table[-1] == "<table>" or table[-1] == "<table><br>" else "td"
        table.append(
            "<tr>" + "".join(f"<{tag}>{c.replace('`', '')}</{tag}>" for c in cells) + "</tr>"
        )
    html = [
        head,
        "<h1>Semantic map from the same replay, seven 2D backends</h1>",
        "<p class=cap>Same patches, projection and fusion for every row; only the 2D model differs. "
        "Rates are against the map's own geometry (height over the local floor, normal direction), "
        "see compare.py. Shares are of labelled voxels.</p>",
        "\n".join(table) + "</table>",
        f"<p>Coarse palette: {legend}</p>",
        f"<h2>Top-down, coarse classes ({name})</h2><div class=grid>",
    ]
    for r, run in zip(rows, runs):
        html.append(
            f"<div><div class=cap>{r['run']}</div><img src='{rel(run / f'coarse_topdown{suffix}.jpg')}'></div>"
        )
    html.append("</div>")
    for qa in common:
        html.append(f"<h2>QA overlay {qa}</h2><div class=grid>")
        for r, run in zip(rows, runs):
            html.append(
                f"<div><div class=cap>{r['run']}</div><img src='{rel(run / 'qa' / qa)}'></div>"
            )
        html.append("</div>")
    (out / "compare.html").write_text("\n".join(html))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_dir")
    ap.add_argument("out_dir")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--name", default="map_semantic")
    ap.add_argument(
        "--lidar-height", type=float, default=0.481, help="pgo's map_clean lidar height"
    )
    ap.add_argument("--voxel", type=float, default=0.1)
    a = ap.parse_args()
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    runs = [Path(r) for r in a.runs]
    xyz0, _, _ = read_labeled(runs[0] / f"{a.name}.pcd")
    geo = Geometry(xyz0, Path(a.map_dir), a.lidar_height, a.voxel)
    print(
        f"geometry from {runs[0].name}: {len(xyz0)} voxels, geo floor {geo.geo_floor.sum()}, wall {geo.geo_wall.sum()}, ceiling {geo.geo_ceiling.sum()}"
    )
    rows, ref = [], None
    for run in runs:
        r, cur = score_run(run, geo, ref, a.name)
        if ref is None:
            ref = cur
        rows.append(r)
        print(
            f"{r['run']:28s} labelled {pct(r['labelled_fraction'])} purity {r['mean_purity']:.2f} "
            f"floor {rate_cell(r['floor_low_horizontal'])} wall {rate_cell(r['wall_vertical'])} "
            f"ceil {rate_cell(r['ceiling_high_horizontal'])} agree {pct(r.get('agreement_with_reference'))}"
        )
    json.dump(rows, open(out / "compare.json", "w"), indent=1)
    write_markdown(rows, out)
    write_html(rows, runs, out, a.name)
    print("wrote", out / "compare.md", out / "compare.html")


if __name__ == "__main__":
    main()
