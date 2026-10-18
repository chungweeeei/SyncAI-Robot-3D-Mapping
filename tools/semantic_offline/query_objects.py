"""Text query over an object map written by objects.py: rank objects by SigLIP 2 similarity to
each query and write a page with the top hits' crops.

    query_objects.py <objects dir> "green sofa" "綠色沙發" ... [--top 6]

SigLIP 2 is multilingual, so Chinese queries go in as they are. Similarity is the cosine of the
text embedding and the object's mean crop embedding; `p` is SigLIP's own sigmoid on it (its
calibrated "this text matches this image" probability), which is the number to threshold on,
not the rank.
"""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path

import numpy as np

from objects import Embedder


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("objects_dir")
    ap.add_argument("queries", nargs="+")
    ap.add_argument("--top", type=int, default=6)
    ap.add_argument("--out", default="query.html")
    a = ap.parse_args()
    d = Path(a.objects_dir)
    meta = json.load(open(d / "objects.json"))
    objs = meta["objects"]
    E = np.load(d / "objects_emb.npy").astype(np.float32)
    emb = Embedder(meta["source"]["embedding_model"])
    # Prompt ensemble: SigLIP was trained on alt-text, and a bare noun phrase scores lower and
    # noisier than the same phrase in a caption. The mean of the normalised template embeddings
    # is the query.
    templates = ["{}", "a photo of a {}.", "a low-angle photo of a {} in a building lobby."]
    T = []
    for q in a.queries:
        t = emb.texts([tp.format(q) for tp in templates]).mean(0)
        T.append(t / np.linalg.norm(t))
    T = np.array(T)
    cos = T @ E.T
    p = emb.probability(cos)
    page = [
        "<!doctype html><meta charset=utf-8><title>Object query</title><style>"
        "body{font:13px system-ui;margin:16px;background:#111;color:#ddd}.r{display:flex;gap:8px;overflow-x:auto}"
        ".c{background:#1c1c1c;padding:6px;min-width:200px}.c img{width:200px;height:150px;object-fit:contain;background:#000}"
        "small{color:#999}</style>"
    ]
    results = {}
    for qi, q in enumerate(a.queries):
        order = np.argsort(-cos[qi])[: a.top]
        print(f'\n"{q}"')
        page.append(f"<h2>{html.escape(q)}</h2><div class=r>")
        results[q] = []
        for rank, i in enumerate(order):
            o = objs[i]
            g = o["goal"]
            print(
                f"  {rank + 1}. #{o['id']:<4d} {o['class']:<14s} {o['color']:<7s} cos {cos[qi, i]:.3f} "
                f"p {p[qi, i]:.3f}  at ({o['centroid'][0]:6.2f}, {o['centroid'][1]:6.2f})  "
                f"goal {'(%.2f, %.2f, %.2f)' % (g['x'], g['y'], g['yaw']) if g else 'none'}"
            )
            results[q].append({"id": o["id"], "cos": float(cos[qi, i]), "p": float(p[qi, i])})
            imgs = "".join(f"<img src='{v['file']}'>" for v in o["views"][:2])
            page.append(
                f"<div class=c>{imgs}<div><b>{rank + 1}. #{o['id']} {o['class']}</b> {o['color']}</div>"
                f"<small>cos {cos[qi, i]:.3f} · p {p[qi, i]:.3f} · ({o['centroid'][0]:.1f}, {o['centroid'][1]:.1f})</small></div>"
            )
        page.append("</div>")
    (d / a.out).write_text("\n".join(page))
    json.dump(
        results, open(d / Path(a.out).with_suffix(".json"), "w"), indent=1, ensure_ascii=False
    )
    print("\nwrote", d / a.out)


if __name__ == "__main__":
    main()
