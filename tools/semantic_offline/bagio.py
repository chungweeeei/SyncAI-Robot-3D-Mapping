"""Bag access for the offline semantic-mapping pipeline: images, camera_info, tf_static and
Livox scans from the rosbag2 directory, without ROS (rosbags + the Livox .msg definitions)."""

from __future__ import annotations

import bisect
import os
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from scipy.spatial.transform import Rotation as R

# The Livox CustomMsg definitions: the Workspace checkout this package is imported into
# (src/syncai_mapping/tools/semantic_offline -> src/third-party/livox_ros_driver2/msg), or LIVOX_MSG_DIR.
LIVOX_MSG_DIR = Path(
    os.environ.get("LIVOX_MSG_DIR")
    or Path(__file__).resolve().parents[3] / "third-party" / "livox_ros_driver2" / "msg"
)

IMAGE_TOPIC = "/robot01/image_raw/compressed"
INFO_TOPIC = "/robot01/camera_info"
LIDAR_TOPIC = "/robot01/livox/lidar"
TF_STATIC_TOPIC = "/tf_static"

# Point-LIO's lidar -> IMU extrinsic (pointlio_params.yaml r_il / t_il): the body frame pgo's
# patches are stored in is the Mid360's IMU frame, p_body = R_il p_lidar + t_il.
R_IL = np.eye(3)
T_IL = np.array([-0.011, -0.02329, 0.04412])


def typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for name in ("CustomPoint", "CustomMsg"):
        add.update(
            get_types_from_msg(
                (LIVOX_MSG_DIR / f"{name}.msg").read_text(), f"livox_ros_driver2/msg/{name}"
            )
        )
    ts.register(add)
    return ts


@dataclass
class Camera:
    width: int
    height: int
    K: np.ndarray  # 3x3
    D: np.ndarray  # plumb_bob k1 k2 p1 p2 k3, or equidistant k1 k2 k3 k4
    frame_id: str
    distortion_model: str = "plumb_bob"


@dataclass
class Transform:
    parent: str
    child: str
    t: np.ndarray  # child origin in parent
    R: np.ndarray  # p_parent = R p_child + t

    def inv(self) -> "Transform":
        return Transform(self.child, self.parent, -self.R.T @ self.t, self.R.T)

    def __matmul__(self, other: "Transform") -> "Transform":
        # self: a<-b, other: b<-c  ->  a<-c
        return Transform(self.parent, other.child, self.R @ other.t + self.t, self.R @ other.R)

    def apply(self, p: np.ndarray) -> np.ndarray:
        return p @ self.R.T + self.t


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def read_static(bag_dir: Path) -> tuple[Camera, dict[tuple[str, str], Transform]]:
    """camera_info and every tf_static transform (keyed by (parent, child))."""
    ts = typestore()
    cam = None
    tfs: dict[tuple[str, str], Transform] = {}
    with Reader(bag_dir) as reader:
        conns = [c for c in reader.connections if c.topic in (INFO_TOPIC, TF_STATIC_TOPIC)]
        for conn, _, raw in reader.messages(connections=conns):
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            if conn.topic == INFO_TOPIC and cam is None:
                cam = Camera(
                    int(msg.width),
                    int(msg.height),
                    np.array(msg.k).reshape(3, 3),
                    np.array(msg.d),
                    msg.header.frame_id,
                    str(getattr(msg, "distortion_model", "") or "plumb_bob"),
                )
            elif conn.topic == TF_STATIC_TOPIC:
                for t in msg.transforms:
                    q = t.transform.rotation
                    tfs[(t.header.frame_id, t.child_frame_id)] = Transform(
                        t.header.frame_id,
                        t.child_frame_id,
                        np.array(
                            [
                                t.transform.translation.x,
                                t.transform.translation.y,
                                t.transform.translation.z,
                            ]
                        ),
                        R.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                    )
            if cam is not None and tfs:
                break
    assert cam is not None, "no camera_info in the bag"
    return cam, tfs


def body_to_camera(tfs: dict[tuple[str, str], Transform], robot_id: str = "robot01") -> Transform:
    """T_cam_body: pointlio body (Mid360 IMU frame) -> camera_optical_frame. lidar_top is taken as
    the Livox lidar frame (the URDF mount), and the IMU sits t_il away from it inside the unit."""
    lidar_cam = tfs[
        (f"{robot_id}/lidar_top", f"{robot_id}/camera_optical_frame")
    ]  # p_lidar = R p_cam + t
    cam_lidar = lidar_cam.inv()
    lidar_body = Transform(
        "lidar", "body", -R_IL.T @ T_IL, R_IL.T
    )  # p_lidar = R_il^T (p_body - t_il)
    return cam_lidar @ lidar_body


class ImageIndex:
    """Stamps (ns) of every image in the bag, and a way to fetch one by index."""

    def __init__(self, bag_dir: Path):
        self.bag_dir = bag_dir
        self.ts = typestore()
        self.stamps: list[int] = []
        self.keys: list[tuple[int, int]] = []  # (recv timestamp, index within topic) for seeking
        with Reader(bag_dir) as reader:
            conns = [c for c in reader.connections if c.topic == IMAGE_TOPIC]
            for i, (conn, recv, raw) in enumerate(reader.messages(connections=conns)):
                msg = self.ts.deserialize_cdr(raw, conn.msgtype)
                self.stamps.append(stamp_ns(msg.header))
                self.keys.append(recv)
        self.stamps_arr = np.array(self.stamps, dtype=np.int64)

    def nearest(self, t_ns: int) -> tuple[int, int]:
        """(index, |dt| ns) of the image stamped nearest to t_ns."""
        i = bisect.bisect_left(self.stamps, t_ns)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self.stamps)]
        j = min(cands, key=lambda j: abs(self.stamps[j] - t_ns))
        return j, abs(self.stamps[j] - t_ns)

    def read_many(self, indices: list[int]):
        """Yield (index, stamp_ns, BGR image) for the requested indices, in bag order (one pass)."""
        want = {self.keys[i]: i for i in indices}
        with Reader(self.bag_dir) as reader:
            conns = [c for c in reader.connections if c.topic == IMAGE_TOPIC]
            for conn, recv, raw in reader.messages(
                connections=conns, start=min(want), stop=max(want) + 1
            ):
                if recv in want:
                    msg = self.ts.deserialize_cdr(raw, conn.msgtype)
                    img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                    yield want[recv], stamp_ns(msg.header), img


def iter_scans(bag_dir: Path, start_ns: int | None = None, stop_ns: int | None = None):
    """Yield (header stamp ns, Nx3 xyz in the lidar frame, N offset ns, reflectivity) per Livox scan."""
    ts = typestore()
    with Reader(bag_dir) as reader:
        conns = [c for c in reader.connections if c.topic == LIDAR_TOPIC]
        for conn, recv, raw in reader.messages(connections=conns, start=start_ns, stop=stop_ns):
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            pts = msg.points
            xyz = (
                np.stack([pts.x, pts.y, pts.z], axis=1).astype(np.float64)
                if hasattr(pts, "x")
                else np.array([[p.x, p.y, p.z] for p in pts])
            )
            off = (
                pts.offset_time
                if hasattr(pts, "offset_time")
                else np.array([p.offset_time for p in pts])
            )
            refl = (
                pts.reflectivity
                if hasattr(pts, "reflectivity")
                else np.array([p.reflectivity for p in pts])
            )
            yield stamp_ns(msg.header), xyz, np.asarray(off), np.asarray(refl)


def project(cam: Camera, p_cam: np.ndarray):
    """Pixel coordinates (N,2) of camera-frame points through K and the camera_info's distortion
    model: plumb_bob (cv2.projectPoints) or equidistant (cv2.fisheye.projectPoints). robot01's
    camera0.yaml is equidistant since its 2026-10-20 calibration (110 x 60 deg on the raw
    stream; the 2026-10-16 bags carried a plumb_bob fit of the stabilized output), and a
    4-coefficient equidistant D fed to projectPoints would be read as k1 k2 p1 p2 and bend
    every point near the border the wrong way."""
    pts = p_cam.reshape(-1, 1, 3).astype(np.float64)
    if cam.distortion_model == "equidistant":
        uv, _ = cv2.fisheye.projectPoints(
            pts, np.zeros(3), np.zeros(3), cam.K, cam.D.reshape(-1, 1)[:4]
        )
    else:
        uv, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), cam.K, cam.D)
    return uv.reshape(-1, 2)


def view_cone(cam: Camera, p_cam: np.ndarray) -> np.ndarray:
    """Mask of the camera-frame points that can land in the image, taken before the (slower)
    projection: in front of the camera and inside the lens's field of view with a margin.
    plumb_bob keeps the per-axis cone the 77 x 44 deg camera was labelled with (the 2026-10-16/17
    runs); equidistant bounds the off-axis angle instead, because a 110 x 60 deg fisheye sees
    ~61 deg off axis at the image corners (r = f * theta, 760 px / 710 px), which |x/z| < 1.1
    (48 deg) cut off together with everything the outer third of the image shows."""
    z = p_cam[:, 2]
    front = z > 0.2
    if cam.distortion_model == "equidistant":
        corners = np.array(
            [[0, 0], [cam.width, 0], [0, cam.height], [cam.width, cam.height]], np.float64
        )
        r_max = np.max(np.hypot(corners[:, 0] - cam.K[0, 2], corners[:, 1] - cam.K[1, 2]))
        theta_max = min(1.15 * r_max / min(cam.K[0, 0], cam.K[1, 1]), 1.4)
        theta = np.arctan2(np.hypot(p_cam[:, 0], p_cam[:, 1]), z)
        return front & (theta < theta_max)
    zz = np.maximum(z, 1e-6)
    return front & (np.abs(p_cam[:, 0] / zz) < 1.1) & (np.abs(p_cam[:, 1] / zz) < 0.8)
