"""One self-contained HTML page with the octomap's occupied voxels coloured by their semantic class
and the object map on top (boxes, labels, a crop per object), for a look in a browser.

    semantic_octomap_viewer.py <map dir> <semantic run dir> <objects dir> <out.html>

The octomap (`octomap.bt`, from clean_map --octomap 1) and the fused semantic voxels share one
grid: an octomap cell of a 0.1 m tree is exactly a fuse.py voxel key, so each occupied cell takes
the class of its semantic voxel (coarse taxonomy, 255 = never in the camera's view). Objects come
from objects.json (objects.py or instances.py) and, when present, objects_voxels.npz: each object
is painted onto the occupied cells it covers, and its box and label are fitted to those cells.
"""

from __future__ import annotations

import base64
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from scipy.spatial import cKDTree

from keyframes import load_poses_txt
from instances import DEFAULT_MAX_DIAG, MAX_DIAG
from objects import LIDAR_HEIGHT
from octomap_io import read_bt
from pcdio import read_labeled
from taxonomy import COARSE, COARSE_PALETTE, UNLABELED, coarse_lut, coarse_of_name

PAINT_NEAR = 0.25  # an occupied cell this close to one of an object's voxels belongs to it, m
# Region growing from the painted cells, bounded per class: the masks' LiDAR points only cover the
# faces the camera saw, so a sofa was painted along one side. Growth follows occupied cells
# (26-neighbours) above the floor, never past the class's size (instances.MAX_DIAG) or height.
GROW_STEPS = 8  # cells, i.e. up to 0.8 m from what the masks hit
# The floor is a 0.1 m layer of cells, so its top cells sit up to ~0.1 m over the floor height:
# growth below GROW_MIN_H walked across the floor between two sofas (one 2 m "sofa" blob).
SEED_MIN_H = 0.10
GROW_MIN_H = 0.15
MAX_HEIGHT = {
    "sofa": 1.3, "armchair": 1.3, "chair": 1.3, "bench": 1.0, "table": 1.2, "counter": 1.4,
    "box": 1.2, "trash can": 1.2, "potted plant": 2.5,
}  # fmt: skip


def vkey(cells: np.ndarray) -> np.ndarray:
    c = cells.astype(np.int64) + (1 << 16)
    return c[:, 0] * (1 << 34) + c[:, 1] * (1 << 17) + c[:, 2]


def main(map_dir: Path, run: Path, objdir: Path, out: Path):
    res, occ, _ = read_bt(map_dir / "octomap.bt")
    print(f"octomap: {len(occ)} occupied cells at {res} m")
    classes = json.load(open(run / "classes.json"))
    lut = coarse_lut({int(k): v for k, v in classes["id2label"].items()})
    xyz, lab, _ = read_labeled(run / "map_semantic_clean.pcd")
    sem_keys = vkey(np.floor(xyz / res).astype(np.int64))
    order = np.argsort(sem_keys)
    sk, sl = sem_keys[order], lut[lab[order]]
    ok = vkey(occ)
    i = np.clip(np.searchsorted(sk, ok), 0, len(sk) - 1)
    coarse = np.where(sk[i] == ok, sl[i], UNLABELED).astype(np.uint8)
    labelled = coarse != UNLABELED
    print(f"labelled cells: {labelled.mean():.1%}")

    # objects painted onto the octomap. An object's own cells (objects_voxels.npz) are the LiDAR
    # points its masks got, ~40 per object and sparse; the occupied cells of its grown box within
    # PAINT_NEAR of them and above the floor are what it actually covers in the octomap. A cell
    # two objects claim goes to the nearer one. Box, label and centroid then come from the painted
    # cells, so they sit on the octomap instead of floating over it.
    meta = json.load(open(objdir / "objects.json"))
    cen = (occ + 0.5) * res
    # Height over the floor the robot walked: keyframe z - lidar height. A local-minimum floor was
    # tried and is wrong on this site: the glossy floor leaves mirror-ghost cells 0.1-0.4 m below
    # it, the minimum found those, and the real floor read as an object to grow into.
    _, kt, _ = load_poses_txt(map_dir / "poses.txt")
    h = cen[:, 2] - (kt[cKDTree(kt[:, :2]).query(cen[:, :2])[1], 2] - LIDAR_HEIGHT)
    obj_of = np.full(len(occ), -1, np.int16)
    best = np.full(len(occ), np.inf, np.float32)
    vox_file = objdir / "objects_voxels.npz"
    vox = np.load(vox_file) if vox_file.exists() else None
    for n, o in enumerate(meta["objects"]):
        if vox is None or f"id_{o['id']}" not in vox.files:
            continue
        vc = (vox[f"id_{o['id']}"] + 0.5) * res
        lo, hi = vc.min(0) - PAINT_NEAR, vc.max(0) + PAINT_NEAR
        cand = np.nonzero(np.all((cen >= lo) & (cen <= hi), axis=1) & (h > SEED_MIN_H))[0]
        if not len(cand):
            continue
        d = cKDTree(vc).query(cen[cand])[0].astype(np.float32)
        win = (d <= PAINT_NEAR) & (d < best[cand])
        obj_of[cand[win]] = n
        best[cand[win]] = d[win]
    n_seed = int((obj_of >= 0).sum())
    index = {int(k): i for i, k in enumerate(vkey(occ))}
    nbr = np.array(
        [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
            if dx or dy or dz
        ]
    )
    for n, o in enumerate(meta["objects"]):
        mine = np.nonzero(obj_of == n)[0]
        if not len(mine):
            continue
        cap = MAX_DIAG.get(o["class"], DEFAULT_MAX_DIAG)
        hmax = MAX_HEIGHT.get(o["class"], 3.0)
        frontier = mine
        lo, hi = cen[mine, :2].min(0), cen[mine, :2].max(0)
        for _ in range(GROW_STEPS):
            cand = (occ[frontier][:, None, :] + nbr[None]).reshape(-1, 3)
            ids = np.array([index.get(int(k), -1) for k in vkey(cand)])
            ids = np.unique(ids[ids >= 0])
            ids = ids[(obj_of[ids] < 0) & (h[ids] > GROW_MIN_H) & (h[ids] < hmax)]
            if not len(ids):
                break
            nlo, nhi = np.minimum(lo, cen[ids, :2].min(0)), np.maximum(hi, cen[ids, :2].max(0))
            if np.linalg.norm(nhi - nlo) > cap:  # keep only what stays inside the size cap
                ok = (
                    np.linalg.norm(
                        np.maximum(hi, cen[ids, :2]) - np.minimum(lo, cen[ids, :2]), axis=1
                    )
                    <= cap
                )
                ids = ids[ok]
                if not len(ids):
                    break
                nlo, nhi = np.minimum(lo, cen[ids, :2].min(0)), np.maximum(hi, cen[ids, :2].max(0))
            obj_of[ids] = n
            lo, hi, frontier = nlo, nhi, ids
    print(
        f"cells painted by an object: {n_seed} from the masks, {(obj_of >= 0).sum()} after growing"
    )

    objs = []
    for n, o in enumerate(meta["objects"]):
        mine = cen[obj_of == n]
        if len(mine):
            bmin, bmax = (mine.min(0) - res / 2).tolist(), (mine.max(0) + res / 2).tolist()
            c = mine.mean(0).tolist()
        else:
            bmin, bmax, c = o["bbox_min"], o["bbox_max"], o["centroid"]
        thumb = ""
        if o["views"]:
            im = cv2.imread(str(objdir / o["views"][0]["file"]))
            if im is not None:
                s = 200 / max(im.shape[:2])
                im = cv2.resize(im, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
                thumb = base64.b64encode(
                    cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]
                ).decode()
        objs.append(
            {
                "id": o["id"],
                "cls": o["class"],
                "color": o["color"],
                "rgb": o["color_rgb_median"],
                "obs": o.get("observations", len(o["views"])),
                "ci": int(coarse_of_name(o["class"])),
                "cells": int(len(mine)),
                "min": bmin,
                "max": bmax,
                "c": c,
                "goal": o["goal"],
                "thumb": thumb,
            }
        )
    counts = {
        COARSE[c]: int((coarse == c).sum()) for c in range(len(COARSE)) if (coarse == c).any()
    }
    data = {
        # the floor the robot walked (median keyframe z - lidar height); a percentile of all cells
        # read 0.17 m low on this site, pulled down by the glossy floor's mirror ghosts
        "floor_z": float(np.median(kt[:, 2]) - LIDAR_HEIGHT),
        "res": res,
        "n": int(len(occ)),
        "labelled": float(labelled.mean()),
        "classes": COARSE,
        "palette": COARSE_PALETTE[: len(COARSE)].tolist(),
        "counts": counts,
        "objects": objs,
        "source": {
            "map": map_dir.name,
            "semantic": run.name,
            "objects": objdir.name,
            "backend": meta.get("source", {}).get("instance_backend", "clustering"),
        },
    }
    cells = occ.astype(np.int16).tobytes()
    page = TEMPLATE.replace("__DATA__", json.dumps(data, ensure_ascii=False))
    page = page.replace("__CELLS__", base64.b64encode(cells).decode())
    page = page.replace("__LABELS__", base64.b64encode(coarse.tobytes()).decode())
    page = page.replace("__OBJ__", base64.b64encode(obj_of.tobytes()).decode())
    out.write_text(page)
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB), {len(objs)} objects")


TEMPLATE = r"""<!doctype html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Semantic OctoMap</title>
<style>
:root{--bg:#f4f5f7;--panel:rgba(255,255,255,.93);--fg:#1c1f24;--muted:#626a75;--line:#d9dde3;--accent:#2f6fde;--box:#d97706}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#101216;--panel:rgba(24,27,33,.93);--fg:#e6e8eb;--muted:#9aa2ad;--line:#2c313a;--accent:#6ea0ff;--box:#fbbf24}}
:root[data-theme="dark"]{--bg:#101216;--panel:rgba(24,27,33,.93);--fg:#e6e8eb;--muted:#9aa2ad;--line:#2c313a;--accent:#6ea0ff;--box:#fbbf24}
*{box-sizing:border-box}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:13px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans TC",sans-serif;overflow:hidden}
#c{position:fixed;inset:0;display:block}
#panel{position:fixed;top:16px;left:16px;width:300px;max-width:calc(100vw - 32px);background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px 16px;backdrop-filter:blur(8px);max-height:calc(100vh - 32px);overflow:auto}
h1{font-size:15px;margin:0 0 2px}
.sub{color:var(--muted);font-size:12px;margin-bottom:8px}
.row{margin:10px 0}
.row label{display:flex;justify-content:space-between;color:var(--muted);font-size:12px;margin-bottom:3px}
.row label b{color:var(--fg);font-weight:600;font-variant-numeric:tabular-nums}
input[type=range]{width:100%;accent-color:var(--accent)}
.chk{display:flex;align-items:center;gap:6px;margin:5px 0;cursor:pointer}
.seg{display:flex;border:1px solid var(--line);border-radius:6px;overflow:hidden}
.seg button{flex:1;border:0;background:transparent;color:var(--fg);padding:5px;cursor:pointer;font:inherit}
.seg button.on{background:var(--accent);color:#fff}
.legend{display:grid;grid-template-columns:1fr 1fr;gap:2px 8px;margin-top:4px}
.lg{display:flex;align-items:center;gap:6px;cursor:pointer;font-size:12px;white-space:nowrap;overflow:hidden}
.lg.off{opacity:.35}
.sw{width:11px;height:11px;border-radius:2px;flex:none;border:1px solid rgba(0,0,0,.2)}
input[type=search]{width:100%;padding:6px 8px;border:1px solid var(--line);border-radius:6px;background:transparent;color:var(--fg);font:inherit}
#list{margin-top:6px;max-height:220px;overflow:auto;border-top:1px solid var(--line)}
.it{padding:4px 2px;border-bottom:1px solid var(--line);cursor:pointer;display:flex;justify-content:space-between;gap:8px}
.it:hover{color:var(--accent)}
.it small{color:var(--muted)}
#info{position:fixed;right:16px;top:16px;width:240px;background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px;display:none;backdrop-filter:blur(8px)}
#info img{width:100%;border-radius:6px;display:block;margin-bottom:8px;background:#000}
#info .k{color:var(--muted)}
#labels{position:fixed;inset:0;pointer-events:none;overflow:hidden}
.tag{position:absolute;transform:translate(-50%,calc(-100% - 7px));font-size:11px;padding:1px 5px;border-radius:4px;background:var(--panel);border:1px solid var(--box);color:var(--fg);white-space:nowrap;pointer-events:auto;cursor:pointer}
.tag::after{content:"";position:absolute;left:50%;bottom:-7px;transform:translateX(-50%);border:6px solid transparent;border-bottom:0;border-top-color:var(--box)}
.tag.sel{background:var(--box);color:#000}
#hint{position:fixed;bottom:12px;left:50%;transform:translateX(-50%);color:var(--muted);font-size:12px;background:var(--panel);padding:4px 10px;border-radius:6px;border:1px solid var(--line)}
@media (max-width:640px){#panel{width:calc(100vw - 32px);max-height:45vh}#info{top:auto;bottom:16px;right:16px;left:16px;width:auto}}
</style>
</head>
<body>
<canvas id="c"></canvas>
<div id="labels"></div>
<div id="panel">
  <h1>Semantic OctoMap</h1>
  <div class="sub" id="sub"></div>
  <div class="row"><div class="seg" id="mode"><button data-m="fused" class="on">融合</button><button data-m="sem">語意體素</button><button data-m="inst">物體實例</button><button data-m="h">高度</button></div></div>
  <div class="row"><label>高度上限（相對地板）<b id="zv"></b></label><input type="range" id="zcut" min="0.2" max="12" step="0.1" value="2.2"></div>
  <div class="row"><label>點大小<b id="pv"></b></label><input type="range" id="psize" min="0.04" max="0.3" step="0.01" value="0.11"></div>
  <label class="chk"><input type="checkbox" id="unl" checked> 顯示未標註體素（灰）</label>
  <label class="chk"><input type="checkbox" id="boxes" checked> 物體框</label>
  <label class="chk"><input type="checkbox" id="tags" checked> 物體標籤</label>
  <div class="row"><label>類別（點擊切換）</label><div class="legend" id="legend"></div></div>
  <div class="row"><label>物體 <b id="ocount"></b></label><input type="search" id="q" placeholder="搜尋類別或顏色，例如 sofa、green"><div id="list"></div></div>
</div>
<div id="info"></div>
<div id="hint">拖曳旋轉 · 右鍵平移 · 滾輪縮放 · 點標籤看物體</div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from "three";
import {OrbitControls} from "three/addons/controls/OrbitControls.js";
const D = __DATA__;
const b64 = s => Uint8Array.from(atob(s), c => c.charCodeAt(0));
const cells = new Int16Array(b64("__CELLS__").buffer);
const labs = b64("__LABELS__");
const objIdx = new Int16Array(b64("__OBJ__").buffer);
const N = D.n, R = D.res;
document.getElementById("sub").textContent = `${D.source.map} · ${N.toLocaleString()} 佔據體素 · ${(D.labelled*100).toFixed(0)}% 有標籤 · 物體：${D.source.backend}`;

const renderer = new THREE.WebGLRenderer({canvas: document.getElementById("c"), antialias: true});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
const setBg = () => scene.background = new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue("--bg").trim() || "#101216");
setBg(); matchMedia("(prefers-color-scheme: dark)").addEventListener("change", setBg);
const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 500);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// positions + floor height (5th percentile of z)
const pos = new Float32Array(N * 3);
let zs = new Float32Array(N);
for (let i = 0; i < N; i++) {
  pos[3*i] = (cells[3*i] + .5) * R; pos[3*i+1] = (cells[3*i+1] + .5) * R; pos[3*i+2] = zs[i] = (cells[3*i+2] + .5) * R;
}
const zFloor = D.floor_z;
const geo = new THREE.BufferGeometry();
geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
const col = new Float32Array(N * 3);
geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
const vis = new Float32Array(N).fill(1);
geo.setAttribute("vis", new THREE.BufferAttribute(vis, 1));
const mat = new THREE.PointsMaterial({size: 0.11, vertexColors: true, sizeAttenuation: true});
mat.onBeforeCompile = sh => {
  sh.vertexShader = "attribute float vis;\nvarying float vVis;\n" + sh.vertexShader.replace("#include <begin_vertex>", "#include <begin_vertex>\nvVis = vis;");
  sh.fragmentShader = "varying float vVis;\n" + sh.fragmentShader.replace("void main() {", "void main() {\nif (vVis < 0.5) discard;");
};
const points = new THREE.Points(geo, mat);
scene.add(points);

const pal = D.palette.map(c => c.map(v => v / 255));
const hidden = new Set();
let mode = "fused";
const instCol = D.objects.map((o, k) => { const c = new THREE.Color().setHSL((k * 0.618034) % 1, 0.75, 0.55); return [c.r, c.g, c.b]; });
function heightColor(t) { // simple turbo-ish ramp
  t = Math.min(1, Math.max(0, t));
  return [Math.min(1, Math.max(0, 1.5 - Math.abs(4*t - 3))), Math.min(1, Math.max(0, 1.5 - Math.abs(4*t - 2))), Math.min(1, Math.max(0, 1.5 - Math.abs(4*t - 1)))];
}
function recolor() {
  const zcut = zFloor + +zcut_el.value, showUnl = unl_el.checked;
  for (let i = 0; i < N; i++) {
    const oi = objIdx[i];
    // fused: an object's class wins on the cells it covers, the voxel semantics elsewhere
    const l = (mode === "fused" || mode === "inst") && oi >= 0 ? D.objects[oi].ci : labs[i];
    let c, v = zs[i] <= zcut;
    if (l === 255) { c = [.55, .55, .58]; v = v && showUnl; }
    else { c = pal[l]; v = v && !hidden.has(l); }
    if (mode === "inst") c = oi >= 0 ? instCol[oi] : [.32, .33, .36];
    if (mode === "h") c = heightColor((zs[i] - zFloor) / 3);
    col[3*i] = c[0]; col[3*i+1] = c[1]; col[3*i+2] = c[2]; vis[i] = v ? 1 : 0;
  }
  geo.attributes.color.needsUpdate = true; geo.attributes.vis.needsUpdate = true;
}
const zcut_el = document.getElementById("zcut"), unl_el = document.getElementById("unl");
const zv = document.getElementById("zv"), pv = document.getElementById("pv");
// A tag sits on the object's top surface, never above the height cut: a door's or a sign's box
// top floated 1-2 m over the voxels on screen, and its parallax made the tag slide off the object
// while orbiting.
// The anchor is one of the object's own visible cells: the one nearest its centroid in plan, then
// the highest visible cell of that column. Anchoring at the centroid left tags floating in the
// hollow of ring-shaped objects (a sofa's frame), up to 0.56 m from any voxel.
const cellsOf = D.objects.map(() => []);
for (let i = 0; i < N; i++) if (objIdx[i] >= 0) cellsOf[objIdx[i]].push(i);
function placeAnchors() {
  const cut = zFloor + +zcut_el.value;
  objEls.forEach((x, n) => {
    const own = cellsOf[n].filter(i => zs[i] <= cut);
    x.hidden = own.length === 0;
    if (x.hidden) return;
    let best = own[0], bd = Infinity;
    for (const i of own) {
      const d = (pos[3*i] - x.o.c[0]) ** 2 + (pos[3*i+1] - x.o.c[1]) ** 2;
      if (d < bd) { bd = d; best = i; }
    }
    let top = best;
    for (const i of own) if (cells[3*i] === cells[3*best] && cells[3*i+1] === cells[3*best+1] && zs[i] > zs[top]) top = i;
    x.top.set(pos[3*top], pos[3*top+1], zs[top] + R / 2);
  });
}
zcut_el.oninput = () => { zv.textContent = (+zcut_el.value).toFixed(1) + " m"; placeAnchors(); recolor(); };
unl_el.onchange = recolor;
document.getElementById("psize").oninput = e => { mat.size = +e.target.value; pv.textContent = (+e.target.value).toFixed(2) + " m"; };
document.querySelectorAll("#mode button").forEach(b => b.onclick = () => {
  mode = b.dataset.m; document.querySelectorAll("#mode button").forEach(x => x.classList.toggle("on", x === b)); recolor();
});
// legend
const legend = document.getElementById("legend");
D.classes.forEach((name, i) => {
  if (!D.counts[name]) return;
  const el = document.createElement("div"); el.className = "lg";
  el.innerHTML = `<span class="sw" style="background:rgb(${D.palette[i].join(",")})"></span>${name} <small style="color:var(--muted)">${D.counts[name] >= 1000 ? (D.counts[name]/1000).toFixed(0) + "k" : D.counts[name]}</small>`;
  el.onclick = () => { hidden.has(i) ? hidden.delete(i) : hidden.add(i); el.classList.toggle("off"); recolor(); };
  legend.appendChild(el);
});

// objects: boxes + tags
const boxColor = getComputedStyle(document.documentElement).getPropertyValue("--box").trim() || "#fbbf24";
const boxGroup = new THREE.Group(); scene.add(boxGroup);
// Tags are sprites in the 3D scene, not HTML on top of it: an HTML overlay has no depth, so a tag
// of an object behind a wall or a sofa still drew in front of everything and read as floating in
// the air. A sprite's bottom edge sits on the object's top cell, it is hidden by voxels in front
// of it, and it is rescaled every frame to a constant size on screen.
const tagGroup = new THREE.Group(); scene.add(tagGroup);
const TAG_PX = 22;  // tag height on screen, CSS px
function makeTag(text, sel) {
  const fs = 40, pad = 14, c = document.createElement("canvas"), g = c.getContext("2d");
  g.font = `600 ${fs}px -apple-system, "Noto Sans TC", sans-serif`;
  c.width = Math.ceil(g.measureText(text).width) + 2 * pad; c.height = fs + 2 * pad + 14;
  g.font = `600 ${fs}px -apple-system, "Noto Sans TC", sans-serif`;
  const h = fs + 2 * pad;
  g.fillStyle = sel ? boxColor : "rgba(24,27,33,0.92)"; g.strokeStyle = boxColor; g.lineWidth = 4;
  g.beginPath(); g.roundRect(2, 2, c.width - 4, h - 4, 10); g.fill(); g.stroke();
  g.beginPath(); g.moveTo(c.width / 2 - 12, h - 2); g.lineTo(c.width / 2 + 12, h - 2); g.lineTo(c.width / 2, c.height); g.closePath(); g.fillStyle = boxColor; g.fill();
  g.fillStyle = sel ? "#000" : "#e6e8eb"; g.textBaseline = "middle"; g.fillText(text, pad, h / 2);
  const tex = new THREE.CanvasTexture(c); tex.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({map: tex, depthTest: true, depthWrite: false}));
  sp.center.set(0.5, 0);  // pivot at the bottom tip: the tip is the anchor
  sp.userData.aspect = c.width / c.height;
  sp.renderOrder = 10;
  return sp;
}
const objEls = [];
let selected = null;
D.objects.forEach(o => {
  const s = new THREE.Vector3(o.max[0]-o.min[0], o.max[1]-o.min[1], o.max[2]-o.min[2]).max(new THREE.Vector3(.08,.08,.08));
  const g = new THREE.EdgesGeometry(new THREE.BoxGeometry(s.x, s.y, s.z));
  const m = new THREE.LineSegments(g, new THREE.LineBasicMaterial({color: boxColor}));
  m.position.set((o.max[0]+o.min[0])/2, (o.max[1]+o.min[1])/2, (o.max[2]+o.min[2])/2);
  boxGroup.add(m);
  const sp = makeTag(`${o.id} ${o.cls}`, false);
  sp.userData.o = o;
  tagGroup.add(sp);
  objEls.push({o, sp, m, top: new THREE.Vector3(o.c[0], o.c[1], o.max[2])});
});
document.getElementById("boxes").onchange = e => boxGroup.visible = e.target.checked;
document.getElementById("tags").onchange = e => tagGroup.visible = e.target.checked;
// click a tag: raycast the sprites
const ray = new THREE.Raycaster(), mouse = new THREE.Vector2();
let downAt = null;
renderer.domElement.addEventListener("pointerdown", e => downAt = [e.clientX, e.clientY]);
renderer.domElement.addEventListener("pointerup", e => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4 || !tagGroup.visible) return;
  mouse.set(e.clientX / innerWidth * 2 - 1, -(e.clientY / innerHeight) * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObjects(tagGroup.children.filter(c => c.visible), false)[0];
  if (hit) select(hit.object.userData.o);
});
const info = document.getElementById("info");
function select(o, fly = false) {
  selected = o;
  objEls.forEach(x => {
    const want = x.o === o;
    if (x.sp.userData.sel !== want) {  // redraw the tag in the selected style
      const n = makeTag(`${x.o.id} ${x.o.cls}`, want);
      x.sp.material.map.dispose(); x.sp.material.map = n.material.map; x.sp.material.needsUpdate = true;
      x.sp.userData.aspect = n.userData.aspect; x.sp.userData.sel = want;
      // the selected tag stays findable: drawn over everything, not hidden by voxels in front
      x.sp.material.depthTest = !want; x.sp.renderOrder = want ? 20 : 10;
    }
    x.m.material.color.set(want ? "#ef4444" : boxColor);
  });
  const g = o.goal ? `(${o.goal.x.toFixed(2)}, ${o.goal.y.toFixed(2)}, yaw ${o.goal.yaw.toFixed(2)})` : "無";
  info.style.display = "block";
  info.innerHTML = (o.thumb ? `<img src="data:image/jpeg;base64,${o.thumb}">` : "") +
    `<b>#${o.id} ${o.cls}</b><br><span class="k">顏色</span> <span class="sw" style="display:inline-block;vertical-align:middle;background:rgb(${o.rgb.join(",")})"></span> ${o.color}` +
    `<br><span class="k">觀測次數</span> ${o.obs} · <span class="k">佔據格</span> ${o.cells}<br><span class="k">中心</span> (${o.c[0].toFixed(2)}, ${o.c[1].toFixed(2)}, ${o.c[2].toFixed(2)})<br><span class="k">導航目標</span> ${g}`;
  if (fly) {
    const c = new THREE.Vector3(...o.c);
    controls.target.copy(c);
    camera.position.set(c.x - 3, c.y - 3, c.z + 2.5);
  }
}
// list + search
const list = document.getElementById("list"), q = document.getElementById("q");
function fillList() {
  const s = q.value.trim().toLowerCase();
  const items = D.objects.filter(o => !s || o.cls.includes(s) || o.color.includes(s) || String(o.id) === s);
  document.getElementById("ocount").textContent = `${items.length} / ${D.objects.length}`;
  list.innerHTML = "";
  items.sort((a, b) => a.cls.localeCompare(b.cls) || b.obs - a.obs).forEach(o => {
    const el = document.createElement("div"); el.className = "it";
    el.innerHTML = `<span>#${o.id} ${o.cls}</span><small>${o.color} · n${o.obs}</small>`;
    el.onclick = () => select(o, true); list.appendChild(el);
  });
}
q.oninput = fillList; fillList();

// camera: fit the map
geo.computeBoundingBox();
const bb = geo.boundingBox, ctr = bb.getCenter(new THREE.Vector3()), size = bb.getSize(new THREE.Vector3());
controls.target.set(ctr.x, ctr.y, zFloor);
camera.position.set(ctr.x, ctr.y - size.y * 0.6, zFloor + Math.max(size.x, size.y) * 0.8);
zv.textContent = (+zcut_el.value).toFixed(1) + " m"; pv.textContent = "0.11 m";
placeAnchors();
recolor();
// deep link: #obj=<id> flies to an object and selects it
const m0 = location.hash.match(/obj=(\d+)/);
if (m0) { const o = D.objects.find(x => x.id === +m0[1]); if (o) select(o, true); }
// #...&az=<deg>: orbit the camera around the target by that azimuth (for side-by-side checks)
const a0 = location.hash.match(/az=(-?\d+)/);
if (a0) {
  const off = camera.position.clone().sub(controls.target), r = Math.hypot(off.x, off.y), t = Math.atan2(off.y, off.x) + (+a0[1]) * Math.PI / 180;
  camera.position.set(controls.target.x + r * Math.cos(t), controls.target.y + r * Math.sin(t), camera.position.z);
}

function resize() { renderer.setSize(innerWidth, innerHeight, false); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); }
addEventListener("resize", resize); resize();
const v = new THREE.Vector3();
function tick() {
  controls.update();
  renderer.render(scene, camera);
  if (tagGroup.visible) {
    const dist = camera.position.distanceTo(controls.target);
    const perPx = 2 * Math.tan(camera.fov * Math.PI / 360) / innerHeight;  // world size of 1 px at distance 1
    for (const x of objEls) {
      const d = camera.position.distanceTo(x.top);
      x.sp.visible = !x.hidden && d < Math.max(12, dist * 1.2);
      if (!x.sp.visible) continue;
      x.sp.position.copy(x.top);
      const hgt = TAG_PX * perPx * d;
      x.sp.scale.set(hgt * x.sp.userData.aspect, hgt, 1);
    }
  }
  requestAnimationFrame(tick);
}
tick();
</script>
</body>
</html>
"""

if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4]))
