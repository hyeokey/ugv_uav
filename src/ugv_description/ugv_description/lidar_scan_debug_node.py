import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class LidarScanDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_scan_debug_node")

        self.declare_parameter("scan_topic", "/ugv/lidar/scan")
        self.declare_parameter("marker_topic", "/ugv/lidar/debug_markers")
        self.declare_parameter("assumed_front_deg", 0.0)
        self.declare_parameter("sector_width_deg", 20.0)
        self.declare_parameter("max_debug_range", 5.0)
        self.declare_parameter("log_period", 1.0)
        self.declare_parameter("publish_markers", True)

        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._marker_topic = str(self.get_parameter("marker_topic").value)
        self._assumed_front = math.radians(float(self.get_parameter("assumed_front_deg").value))
        self._sector_width = math.radians(abs(float(self.get_parameter("sector_width_deg").value)))
        self._max_debug_range = abs(float(self.get_parameter("max_debug_range").value))
        self._log_period = abs(float(self.get_parameter("log_period").value))
        self._publish_markers = _as_bool(self.get_parameter("publish_markers").value)
        self._last_log_monotonic = 0.0

        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f"LiDAR scan debug listening to {self._scan_topic}; "
            f"assuming front={math.degrees(self._assumed_front):.1f} deg in scan frame"
        )

    def _valid_range(self, msg: LaserScan, range_value: float) -> bool:
        return math.isfinite(range_value) and msg.range_min <= range_value <= msg.range_max

    def _scan_angle_at_index(self, msg: LaserScan, index: int) -> float:
        return msg.angle_min + index * msg.angle_increment

    def _nearest_in_sector(self, msg: LaserScan, center_angle: float, width: float) -> Tuple[float, Optional[float]]:
        half_width = 0.5 * width
        nearest = float("inf")
        nearest_angle: Optional[float] = None
        angle = msg.angle_min

        for range_value in msg.ranges:
            if self._valid_range(msg, range_value):
                error = _wrap_pi(angle - center_angle)
                if abs(error) <= half_width and range_value < nearest:
                    nearest = float(range_value)
                    nearest_angle = angle
            angle += msg.angle_increment

        if not math.isfinite(nearest):
            return float("nan"), None
        return nearest, nearest_angle

    def _range_at_angle(self, msg: LaserScan, target_angle: float) -> Tuple[float, Optional[float]]:
        if msg.angle_increment == 0.0 or not msg.ranges:
            return float("nan"), None

        raw_index = int(round((target_angle - msg.angle_min) / msg.angle_increment))
        if raw_index < 0 or raw_index >= len(msg.ranges):
            return float("nan"), None

        range_value = float(msg.ranges[raw_index])
        if not self._valid_range(msg, range_value):
            return float("nan"), self._scan_angle_at_index(msg, raw_index)
        return range_value, self._scan_angle_at_index(msg, raw_index)

    def _make_arrow(self, marker_id: int, frame_id: str, angle: float, length: float, color) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = "lidar_debug_axes"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.045
        marker.scale.y = 0.11
        marker.scale.z = 0.11
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 0.95
        start = Point()
        end = Point()
        end.x = float(length * math.cos(angle))
        end.y = float(length * math.sin(angle))
        marker.points = [start, end]
        return marker

    def _make_point(self, marker_id: int, frame_id: str, angle: Optional[float], distance: float, color) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        marker.ns = "lidar_debug_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 0.95

        if angle is None or not math.isfinite(distance):
            marker.action = Marker.DELETE
            return marker

        marker.action = Marker.ADD
        marker.pose.position.x = float(distance * math.cos(angle))
        marker.pose.position.y = float(distance * math.sin(angle))
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        return marker

    def _publish_debug_markers(
        self,
        frame_id: str,
        front_range: float,
        front_angle: Optional[float],
        left_range: float,
        left_angle: Optional[float],
        right_range: float,
        right_angle: Optional[float],
    ) -> None:
        if not self._publish_markers:
            return

        markers = MarkerArray()
        front = self._assumed_front
        left = _wrap_pi(front + math.pi * 0.5)
        right = _wrap_pi(front - math.pi * 0.5)
        length = self._max_debug_range

        markers.markers.append(self._make_arrow(0, frame_id, front, length, (1.0, 0.0, 0.0)))
        markers.markers.append(self._make_arrow(1, frame_id, left, length * 0.7, (0.0, 1.0, 0.0)))
        markers.markers.append(self._make_arrow(2, frame_id, right, length * 0.7, (0.0, 0.2, 1.0)))
        markers.markers.append(self._make_point(10, frame_id, front_angle, front_range, (1.0, 0.0, 0.0)))
        markers.markers.append(self._make_point(11, frame_id, left_angle, left_range, (0.0, 1.0, 0.0)))
        markers.markers.append(self._make_point(12, frame_id, right_angle, right_range, (0.0, 0.2, 1.0)))
        self._marker_pub.publish(markers)

    def _on_scan(self, msg: LaserScan) -> None:
        front_center = self._assumed_front
        left_center = _wrap_pi(front_center + math.pi * 0.5)
        right_center = _wrap_pi(front_center - math.pi * 0.5)

        front_exact, front_exact_angle = self._range_at_angle(msg, front_center)
        left_exact, left_exact_angle = self._range_at_angle(msg, left_center)
        right_exact, right_exact_angle = self._range_at_angle(msg, right_center)

        front_nearest, front_nearest_angle = self._nearest_in_sector(msg, front_center, self._sector_width)
        left_nearest, left_nearest_angle = self._nearest_in_sector(msg, left_center, self._sector_width)
        right_nearest, right_nearest_angle = self._nearest_in_sector(msg, right_center, self._sector_width)

        self._publish_debug_markers(
            msg.header.frame_id,
            front_nearest,
            front_nearest_angle,
            left_nearest,
            left_nearest_angle,
            right_nearest,
            right_nearest_angle,
        )

        now = time.monotonic()
        if now - self._last_log_monotonic < self._log_period:
            return

        self._last_log_monotonic = now
        self.get_logger().info(
            f"scan frame={msg.header.frame_id} angle=[{math.degrees(msg.angle_min):.1f}, "
            f"{math.degrees(msg.angle_max):.1f}] deg inc={math.degrees(msg.angle_increment):.2f} deg "
            f"count={len(msg.ranges)}"
        )
        self.get_logger().info(
            f"exact 0/+90/-90 ranges: front={front_exact:.2f}@{math.degrees(front_exact_angle) if front_exact_angle is not None else float('nan'):.1f}deg "
            f"left={left_exact:.2f}@{math.degrees(left_exact_angle) if left_exact_angle is not None else float('nan'):.1f}deg "
            f"right={right_exact:.2f}@{math.degrees(right_exact_angle) if right_exact_angle is not None else float('nan'):.1f}deg"
        )
        self.get_logger().info(
            f"nearest sectors width={math.degrees(self._sector_width):.1f}deg: "
            f"front={front_nearest:.2f}@{math.degrees(front_nearest_angle) if front_nearest_angle is not None else float('nan'):.1f}deg "
            f"left={left_nearest:.2f}@{math.degrees(left_nearest_angle) if left_nearest_angle is not None else float('nan'):.1f}deg "
            f"right={right_nearest:.2f}@{math.degrees(right_nearest_angle) if right_nearest_angle is not None else float('nan'):.1f}deg"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarScanDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
