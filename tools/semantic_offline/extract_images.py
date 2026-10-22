"""Build a small image-only rosbag2 from a zstd file-compressed recording, one file at a time: the
image nearest each keyframe stamp (plus camera_info and tf_static), with the original stamps and
receive times, so bagio.ImageIndex reads it exactly as it would read the full bag.

    extract_images.py <compressed bag dir> <out bag dir> --keyframes <keyframes.txt> [--window-ms 60]
                      [--every-s 0]

Why: a 20-minute recording with a 30 Hz camera is ~14 GB decompressed (JPEG does not compress),
rosbags' Reader inflates every file of a file-compressed bag at open, and the labelling needs one
image per keyframe (~2 000 of 35 000). Each .db3.zstd is inflated into a scratch view (one file,
its own metadata.yaml), read, and deleted before the next; peak disk use is one file. A keyframe
whose stamp lies within --window-ms of a file boundary may get its nearest image from each of the
two files; both are written and ImageIndex.nearest() picks the nearer. --every-s keeps one image
every N seconds on top (0 = off), for tools that want views between keyframes."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml
from rosbags.rosbag2 import Reader, Writer

from bagio import IMAGE_TOPIC, INFO_TOPIC, TF_STATIC_TOPIC, stamp_ns, typestore


def view_for(bag_dir: Path, meta: dict, index: int, view_dir: Path) -> Path:
    """Inflate file <index> of the compressed bag into view_dir as a one-file uncompressed bag."""
    if view_dir.exists():
        shutil.rmtree(view_dir)
    view_dir.mkdir(parents=True)
    info = meta["rosbag2_bagfile_information"]
    src = bag_dir / info["relative_file_paths"][index]
    dst = view_dir / re.sub(r"\.zstd$", "", src.name)
    subprocess.run(["zstd", "-d", "-q", "-f", "-o", str(dst), str(src)], check=True)
    one = {
        "rosbag2_bagfile_information": {
            **info,
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [dst.name],
            "files": [
                {**info["files"][index], "path": dst.name}
            ]
            if info.get("files")
            else [],
        }
    }
    (view_dir / "metadata.yaml").write_text(yaml.safe_dump(one, sort_keys=False))
    return view_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--keyframes", required=True, help="keyframes.txt (name stamp_ns x y z qw qx qy qz)")
    ap.add_argument("--window-ms", type=float, default=60.0)
    ap.add_argument("--every-s", type=float, default=0.0)
    ap.add_argument("--view-dir", default="", help="scratch dir for the inflated file (default: <out>_view)")
    a = ap.parse_args()
    bag, out = Path(a.bag), Path(a.out)
    view_dir = Path(a.view_dir) if a.view_dir else out.with_name(out.name + "_view")
    meta = yaml.safe_load((bag / "metadata.yaml").read_text())
    files = meta["rosbag2_bagfile_information"]["relative_file_paths"]
    kf = np.array(
        [int(line.split()[1]) for line in Path(a.keyframes).read_text().splitlines() if line.strip()],
        dtype=np.int64,
    )
    kf.sort()
    win = int(a.window_ms * 1e6)
    ts = typestore()
    if out.exists():
        sys.exit(f"{out} exists; remove it first")
    n_img = n_kept = 0
    last_every = -1
    with Writer(out, version=8) as w:
        conn_out = {}
        for fi in range(len(files)):
            view = view_for(bag, meta, fi, view_dir)
            print(f"file {fi}: {files[fi]}", flush=True)
            with Reader(view) as r:
                conns = {c.topic: c for c in r.connections}
                for topic in (INFO_TOPIC, TF_STATIC_TOPIC, IMAGE_TOPIC):
                    if topic in conns and topic not in conn_out:
                        conn_out[topic] = w.add_connection(
                            topic,
                            conns[topic].msgtype,
                            typestore=ts,
                            serialization_format=conns[topic].ext.serialization_format,
                            offered_qos_profiles=conns[topic].ext.offered_qos_profiles,
                        )
                # image stamps of this file, then the nearest one per keyframe inside the window
                stamps, recvs, raws = [], [], []
                static = []
                for c, recv, raw in r.messages(
                    connections=[conns[t] for t in (INFO_TOPIC, TF_STATIC_TOPIC, IMAGE_TOPIC) if t in conns]
                ):
                    if c.topic != IMAGE_TOPIC:
                        static.append((c.topic, recv, raw))
                        continue
                    msg = ts.deserialize_cdr(raw, c.msgtype)
                    stamps.append(stamp_ns(msg.header))
                    recvs.append(recv)
                    raws.append(raw)
                for topic, recv, raw in static:
                    w.write(conn_out[topic], recv, raw)
                st = np.array(stamps, dtype=np.int64)
                keep = np.zeros(len(st), bool)
                if len(st):
                    j = np.searchsorted(st, kf)
                    for k, jj in zip(kf, j):
                        cands = [x for x in (jj - 1, jj) if 0 <= x < len(st)]
                        if not cands:
                            continue
                        best = min(cands, key=lambda x: abs(int(st[x]) - int(k)))
                        if abs(int(st[best]) - int(k)) <= win:
                            keep[best] = True
                    if a.every_s > 0:
                        slot = (st // int(a.every_s * 1e9)).astype(np.int64)
                        first = np.r_[True, slot[1:] != slot[:-1]]
                        first &= slot != last_every
                        keep |= first
                        last_every = int(slot[-1])
                for i in np.nonzero(keep)[0]:
                    w.write(conn_out[IMAGE_TOPIC], recvs[i], raws[i])
                n_img += len(st)
                n_kept += int(keep.sum())
                print(f"  {len(st)} images, kept {int(keep.sum())} ({n_kept} so far)", flush=True)
            shutil.rmtree(view_dir)
    print(f"wrote {out}: {n_kept} of {n_img} images for {len(kf)} keyframes")


if __name__ == "__main__":
    main()
