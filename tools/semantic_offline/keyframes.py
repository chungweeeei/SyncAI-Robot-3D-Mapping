"""Reproduce pgo_node's keyframe sequence with stamps from the replay recording (poses.txt has no
stamps): re-run isKeyPose over the pairs pgo processed (one map->pointlio_odom TF per processed
pair), then check the result against poses.txt by count and by consecutive relative motion."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

KEYFRAME_DELTA_M = 0.5
KEYFRAME_DELTA_DEG = 10.0
RAD2DEG = 57.324  # pose_graph_optimizer.cpp uses this constant, not 180/pi


def load_pose_file(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """stamp ns, t (N,3), q (N,4 xyzw) from '<ns> x y z qw qx qy qz' lines."""
    rows = np.array(
        [[float(x) for x in line.split()] for line in path.read_text().splitlines() if line.strip()]
    )
    ns = np.array(
        [int(line.split()[0]) for line in path.read_text().splitlines() if line.strip()],
        dtype=np.int64,
    )
    t = rows[:, 1:4]
    q = rows[:, [5, 6, 7, 4]]  # xyzw
    return ns, t, q


def load_poses_txt(path: Path) -> tuple[list[str], np.ndarray, np.ndarray]:
    names, ts, qs = [], [], []
    for line in path.read_text().splitlines():
        p = line.split()
        if len(p) < 8:
            continue
        names.append(p[0])
        ts.append([float(x) for x in p[1:4]])
        qw, qx, qy, qz = (float(x) for x in p[4:8])
        qs.append([qx, qy, qz, qw])
    return names, np.array(ts), np.array(qs)


def select_keyframes(odom_ns, odom_t, odom_q, processed_ns) -> list[int]:
    """Indices into the odom arrays of the keyframes, applying isKeyPose over processed pairs."""
    idx_of = {int(s): i for i, s in enumerate(odom_ns)}
    keys = []
    last_t = last_R = None
    for s in processed_ns:
        i = idx_of.get(int(s))
        if i is None:
            continue
        if last_t is None:
            keys.append(i)
            last_t, last_R = odom_t[i], R.from_quat(odom_q[i])
            continue
        d = np.linalg.norm(odom_t[i] - last_t)
        ang = (last_R.inv() * R.from_quat(odom_q[i])).magnitude() * RAD2DEG
        if d > KEYFRAME_DELTA_M or ang > KEYFRAME_DELTA_DEG:
            keys.append(i)
            last_t, last_R = odom_t[i], R.from_quat(odom_q[i])
    return keys


def relative(t, q, i, j):
    Ri = R.from_quat(q[i])
    return Ri.inv().apply(t[j] - t[i]), (Ri.inv() * R.from_quat(q[j])).magnitude()


def main(replay_dir: Path, map_dir: Path, out: Path):
    odom_ns, odom_t, odom_q = load_pose_file(replay_dir / "lio_odom.txt")
    tf_ns, _, _ = load_pose_file(replay_dir / "tf_map_odom.txt")
    names, kt, kq = load_poses_txt(map_dir / "poses.txt")
    keys = select_keyframes(odom_ns, odom_t, odom_q, tf_ns)
    print(
        f"odom {len(odom_ns)} processed pairs {len(tf_ns)} -> {len(keys)} keyframes; poses.txt has {len(names)}"
    )
    n = min(len(keys), len(names))
    # consecutive relative motion, odom vs world: equal up to loop corrections
    worst = 0.0
    bad = []
    for k in range(1, n):
        dt_o, da_o = relative(odom_t, odom_q, keys[k - 1], keys[k])
        dt_w, da_w = relative(kt, kq, k - 1, k)
        e = np.linalg.norm(dt_o - dt_w)
        worst = max(worst, e)
        if e > 0.05 or abs(da_o - da_w) > 0.05:
            bad.append((k, e, abs(da_o - da_w)))
    print(
        f"relative-motion check over {n} keyframes: worst translation mismatch {worst:.3f} m, {len(bad)} above 5 cm / 0.05 rad"
    )
    for b in bad[:10]:
        print("  mismatch at keyframe", b)
    if len(keys) != len(names):
        print("COUNT MISMATCH: keyframe reproduction is not exact; inspect before labelling")
    with open(out, "w") as f:
        for k in range(n):
            i = keys[k]
            f.write(
                f"{names[k]} {odom_ns[i]} {odom_t[i, 0]:.6f} {odom_t[i, 1]:.6f} {odom_t[i, 2]:.6f} "
                f"{odom_q[i, 3]:.8f} {odom_q[i, 0]:.8f} {odom_q[i, 1]:.8f} {odom_q[i, 2]:.8f}\n"
            )
    print("wrote", out)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
