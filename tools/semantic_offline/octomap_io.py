"""Read an OctoMap binary tree (`octomap.bt`, OcTree::writeBinary) without the octomap library:
the occupied (and optionally free) leaves as full-resolution voxel keys.

Format (octomap's OcTreeBase::writeBinaryNode): an ASCII header ending in a `data` line, then
the tree depth-first from the root. Every node writes two bytes holding two bits per child
(children 0-3, then 4-7), written as bit[2i] bit[2i+1]: 00 none (unknown), 01 occupied leaf,
10 free leaf, 11 inner node;
the inner children follow, in child order. Child i sits at +x if i & 1, +y if i & 2, +z if i & 4.

Keys are octomap's: a leaf at depth 16 has key k per axis, its cell spans
[(k - 32768) * res, (k - 32768 + 1) * res), i.e. the same grid as floor(x / res). That is what
lets a voxel key from fuse.py (0.1 m) index an octomap cell of a 0.1 m tree directly.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

TREE_DEPTH = 16
TREE_MAX = 32768


def read_bt(path: Path, want_free: bool = False):
    """-> (res, occupied int32 (N,3) cell indices, free int32 (M,3) or None). Cell index =
    key - 32768, so the cell's centre is (index + 0.5) * res. Pruned leaves above the finest depth
    are expanded into all their cells."""
    data = Path(path).read_bytes()
    pos = 0
    res = None
    while True:
        end = data.index(b"\n", pos)
        line = data[pos:end].decode("ascii", "replace").strip()
        pos = end + 1
        if line.startswith("res"):
            res = float(line.split()[1])
        if line == "data":
            break
    assert res is not None, f"{path}: no 'res' in the header"
    occ_blocks, free_blocks = [], []  # (key_min xyz, side in cells)
    # explicit stack: (depth, centre key xyz) of inner nodes still to read, in file order
    stack = [(0, (TREE_MAX, TREE_MAX, TREE_MAX))]
    while stack:
        depth, key = stack.pop()
        b0, b1 = data[pos], data[pos + 1]
        pos += 2
        bits = b0 | (b1 << 8)
        off = TREE_MAX >> (depth + 1)
        children = []
        for i in range(8):
            code = (bits >> (2 * i)) & 3
            if code == 0:
                continue
            ck = tuple(
                key[a] + off if (i >> a) & 1 else key[a] - off - (0 if off else 1) for a in range(3)
            )
            if code == 3:
                children.append((depth + 1, ck))
                continue
            side = 1 << (TREE_DEPTH - depth - 1)  # cells per axis of this leaf
            # a node's centre key at depth d < 16 is the first key of its upper half
            kmin = tuple(c - side // 2 for c in ck) if side > 1 else ck
            # code = bit[2i] | bit[2i+1] << 1: octomap writes an occupied leaf as bit[2i] = 0,
            # bit[2i+1] = 1, i.e. 2, and a free one as 1 (reading them the other way round turned
            # the free space into 15.9 M "occupied" cells)
            if code == 2:
                occ_blocks.append((kmin, side))
            elif want_free:
                free_blocks.append((kmin, side))
        stack.extend(reversed(children))  # file order: children 0..7, depth first

    def expand(blocks):
        out = []
        for kmin, side in blocks:
            if side == 1:
                out.append(np.array([kmin], np.int32))
            else:
                r = np.arange(side, dtype=np.int32)
                g = np.stack(np.meshgrid(r, r, r, indexing="ij"), -1).reshape(-1, 3)
                out.append(g + np.array(kmin, np.int32))
        return (np.concatenate(out) - TREE_MAX) if out else np.zeros((0, 3), np.int32)

    return res, expand(occ_blocks), (expand(free_blocks) if want_free else None)


if __name__ == "__main__":
    import sys

    r, occ, _ = read_bt(Path(sys.argv[1]))
    c = (occ + 0.5) * r
    print(f"res {r}, {len(occ)} occupied cells, extent {c.min(0).round(2)} .. {c.max(0).round(2)}")
