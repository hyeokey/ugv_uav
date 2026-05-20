import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class LaserScanToPointCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("laser_scan_to_pointcloud_node")
        self.declare_parameter("scan_topic", "/ugv/lidar/scan")
        self.declare_parameter("points_topic", "/ugv/lidar/points")

        scan_topic = self.get_parameter("scan_topic").value
        points_topic = self.get_parameter("points_topic").value

        self._points_pub = self.create_publisher(PointCloud2, points_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(f"Publishing PointCloud2 {points_topic} from {scan_topic}")

    def _on_scan(self, msg: LaserScan) -> None:
        points = []
        angle = msg.angle_min
        for range_value in msg.ranges:
            if math.isfinite(range_value) and msg.range_min <= range_value <= msg.range_max:
                points.append((
                    range_value * math.cos(angle),
                    range_value * math.sin(angle),
                    0.0,
                ))
            angle += msg.angle_increment

        cloud = point_cloud2.create_cloud_xyz32(msg.header, points)
        self._points_pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
