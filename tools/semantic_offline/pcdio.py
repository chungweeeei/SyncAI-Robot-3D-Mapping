"""Minimal PCD I/O: pgo's binary 'x y z intensity' patches in, and a binary PCD with a label field out."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def read_xyz(path: Path) -> np.ndarray:
    """(N,3) float32 from a binary PCD whose fields start with x y z (float32); row order kept."""
    with open(path, "rb") as f:
        fields = sizes = types = counts = None
        points = 0
        while True:
            line = f.readline().decode("ascii", "replace")
            key, _, rest = line.strip().partition(" ")
            if key == "FIELDS":
                fields = rest.split()
            elif key == "SIZE":
                sizes = [int(x) for x in rest.split()]
            elif key == "TYPE":
                types = rest.split()
            elif key == "COUNT":
                counts = [int(x) for x in rest.split()]
            elif key == "POINTS":
                points = int(rest)
            elif key == "DATA":
                assert rest.strip() == "binary", f"{path}: only DATA binary is supported"
                break
        dtype = np.dtype(
            [
                (n, {"F": "f", "U": "u", "I": "i"}[t] + str(s))
                if c == 1
                else (n, {"F": "f", "U": "u", "I": "i"}[t] + str(s), (c,))
                for n, s, t, c in zip(fields, sizes, types, counts)
            ]
        )
        arr = np.frombuffer(f.read(points * dtype.itemsize), dtype=dtype, count=points)
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)


def write_labeled(
    path: Path,
    xyz: np.ndarray,
    rgb: np.ndarray,
    label: np.ndarray,
    weight: np.ndarray | None = None,
):
    """Binary PCD with x y z (f32), rgb (packed f32, PCL style), label (u8 as u32 for PCL), optional
    confidence (f32)."""
    n = len(xyz)
    packed = (
        (rgb[:, 0].astype(np.uint32) << 16)
        | (rgb[:, 1].astype(np.uint32) << 8)
        | rgb[:, 2].astype(np.uint32)
    )
    fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4"), ("label", "<u4")]
    if weight is not None:
        fields.append(("confidence", "<f4"))
    arr = np.empty(n, dtype=np.dtype(fields))
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr["rgb"] = packed.view(np.float32)
    arr["label"] = label.astype(np.uint32)
    if weight is not None:
        arr["confidence"] = weight.astype(np.float32)
    names = " ".join(f[0] for f in fields)
    sizes = " ".join("4" for _ in fields)
    types = " ".join("U" if f[0] == "label" else "F" for f in fields)
    counts = " ".join("1" for _ in fields)
    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS {names}\nSIZE {sizes}\n"
        f"TYPE {types}\nCOUNT {counts}\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(arr.tobytes())


def read_labeled(path: Path):
    """Read back a PCD written by write_labeled: (xyz f32 Nx3, label uint8, confidence f32)."""
    with open(path, "rb") as f:
        fields = None
        while True:
            line = f.readline().decode("ascii", "replace")
            key, _, rest = line.strip().partition(" ")
            if key == "FIELDS":
                fields = rest.split()
            elif key == "POINTS":
                n = int(rest)
            elif key == "DATA":
                break
        dtype = np.dtype([(name, "<u4" if name == "label" else "<f4") for name in fields])
        arr = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
    conf = arr["confidence"] if "confidence" in fields else np.ones(n, np.float32)
    return xyz, arr["label"].astype(np.uint8), conf
