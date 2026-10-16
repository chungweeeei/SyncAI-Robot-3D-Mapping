"""Record every processed pair's stamp (pgo's map->pointlio_odom TF) and the LIO odometry, so the
keyframe sequence pgo_node chose can be reproduced offline with stamps (poses.txt has none)."""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

out = sys.argv[1]


class Recorder(Node):
    def __init__(self):
        super().__init__("kf_recorder")
        qos = QoSProfile(
            depth=10000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
        )
        self.fo = open(f"{out}/lio_odom.txt", "w")
        self.ft = open(f"{out}/tf_map_odom.txt", "w")
        self.fm = open(f"{out}/loop_markers.txt", "w")
        self.create_subscription(Odometry, "/robot01/pointlio/lio_odom", self.odom, qos)
        self.create_subscription(TFMessage, "/tf", self.tf, qos)
        # keyframe ticks after the first loop: header stamp = the keyframe's stamp, points = all nodes
        self.create_subscription(MarkerArray, "/robot01/pgo/loop_markers", self.markers, qos)
        self.n = [0, 0, 0]
        self.create_timer(30.0, self.report)

    def odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        self.fo.write(
            f"{ns} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.w:.8f} {q.x:.8f} {q.y:.8f} {q.z:.8f}\n"
        )
        self.n[0] += 1

    def tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "robot01/pointlio_odom":
                tr, q = t.transform.translation, t.transform.rotation
                ns = t.header.stamp.sec * 10**9 + t.header.stamp.nanosec
                self.ft.write(
                    f"{ns} {tr.x:.6f} {tr.y:.6f} {tr.z:.6f} {q.w:.8f} {q.x:.8f} {q.y:.8f} {q.z:.8f}\n"
                )
                self.n[1] += 1

    def markers(self, m):
        for mk in m.markers:
            if mk.ns == "pgo_nodes":
                ns = mk.header.stamp.sec * 10**9 + mk.header.stamp.nanosec
                self.fm.write(f"{ns} {len(mk.points)}\n")
                self.n[2] += 1

    def report(self):
        self.get_logger().info(f"odom {self.n[0]} tf {self.n[1]} markers {self.n[2]}")
        for f in (self.fo, self.ft, self.fm):
            f.flush()


rclpy.init()
node = Recorder()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    for f in (node.fo, node.ft, node.fm):
        f.close()
