import math
from typing import Iterable, List

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def _matmul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def _normalize_quat_wxyz(q: Iterable[float]) -> List[float]:
    q = [float(v) for v in q]
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("invalid quaternion")
    return [v / norm for v in q]


def _quat_wxyz_to_matrix(q: Iterable[float]) -> List[List[float]]:
    w, x, y, z = _normalize_quat_wxyz(q)
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _matrix_to_quat_xyzw(m: List[List[float]]) -> List[float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return [x / norm, y / norm, z / norm, w / norm]


def _px4_position_ned_to_enu(position: Iterable[float]) -> List[float]:
    north, east, down = [float(v) for v in position]
    return [east, north, -down]


def _px4_body_frd_to_ros_body_flu_orientation(q_wxyz: Iterable[float]) -> List[float]:
    # PX4 VehicleOdometry q maps body FRD into local NED. RViz wants body FLU in ENU.
    ned_to_enu = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    flu_to_frd = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    r_ned_frd = _quat_wxyz_to_matrix(q_wxyz)
    r_enu_flu = _matmul(_matmul(ned_to_enu, r_ned_frd), flu_to_frd)
    return _matrix_to_quat_xyzw(r_enu_flu)


class Px4OdometryTfNode(Node):
    def __init__(self) -> None:
        super().__init__("px4_odometry_tf_node")

        self.declare_parameter("odometry_topic", "/px4_1/fmu/out/vehicle_odometry")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("publish_2d", True)
        self.declare_parameter("zero_z", True)

        self._parent_frame = self.get_parameter("parent_frame").value
        self._child_frame = self.get_parameter("child_frame").value
        self._publish_2d = bool(self.get_parameter("publish_2d").value)
        self._zero_z = bool(self.get_parameter("zero_z").value)

        try:
            from px4_msgs.msg import VehicleOdometry
        except ImportError as exc:
            raise RuntimeError(
                "px4_msgs is not sourced. Source the workspace that provides px4_msgs before "
                "running this node, for example: source ~/drone_space/ros2_ws/install/setup.bash"
            ) from exc

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        topic = self.get_parameter("odometry_topic").value
        self.create_subscription(VehicleOdometry, topic, self._on_odometry, qos)
        self.get_logger().info(f"Publishing TF {self._parent_frame} -> {self._child_frame} from {topic}")

    def _on_odometry(self, msg) -> None:
        try:
            xyz = _px4_position_ned_to_enu(msg.position)
            q_xyzw = _px4_body_frd_to_ros_body_flu_orientation(msg.q)
        except ValueError:
            self.get_logger().warn("Skipping odometry with invalid quaternion", throttle_duration_sec=2.0)
            return

        if self._zero_z:
            xyz[2] = 0.0

        if self._publish_2d:
            yaw = math.atan2(
                2.0 * (q_xyzw[3] * q_xyzw[2] + q_xyzw[0] * q_xyzw[1]),
                1.0 - 2.0 * (q_xyzw[1] * q_xyzw[1] + q_xyzw[2] * q_xyzw[2]),
            )
            q_xyzw = [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)]

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        transform.transform.rotation.x = q_xyzw[0]
        transform.transform.rotation.y = q_xyzw[1]
        transform.transform.rotation.z = q_xyzw[2]
        transform.transform.rotation.w = q_xyzw[3]
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4OdometryTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
