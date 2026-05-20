import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def _dictionary(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module is not available")

    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise RuntimeError(f"Unknown ArUco dictionary: {name}")

    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    return cv2.aruco.Dictionary_get(dictionary_id)


def _detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()

    return cv2.aruco.DetectorParameters_create()


def _detect_markers(gray, dictionary, parameters):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(gray)

    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def _image_to_bgr(msg: Image):
    channels_by_encoding = {
        "mono8": 1,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }
    encoding = msg.encoding.lower()
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    data = np.frombuffer(msg.data, dtype=np.uint8)
    image = data.reshape((msg.height, msg.width, channels))

    if encoding == "mono8":
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    return image


def _rotation_matrix_to_quaternion(rotation):
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s

    return qx, qy, qz, qw


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector_node")
        self.declare_parameter("image_topic", "/drone/down_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/drone/down_camera/camera_info")
        self.declare_parameter("pose_topic", "/drone/aruco/pose")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.32)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("marker_frame", "rover_aruco_marker")

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        self._marker_id = int(self.get_parameter("marker_id").value)
        self._marker_size = float(self.get_parameter("marker_size").value)
        dictionary_name = self.get_parameter("dictionary").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._marker_frame = self.get_parameter("marker_frame").value

        self._dictionary = _dictionary(dictionary_name)
        self._parameters = _detector_parameters()
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._camera_frame = ""
        self._last_warn_time = self.get_clock().now()

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"Detecting ArUco id {self._marker_id} ({dictionary_name}, {self._marker_size:.3f} m) "
            f"from {image_topic}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self._camera_frame = msg.header.frame_id

    def _on_image(self, msg: Image) -> None:
        if self._camera_matrix is None or self._dist_coeffs is None:
            now = self.get_clock().now()
            if (now - self._last_warn_time).nanoseconds > 2_000_000_000:
                self.get_logger().warn("Waiting for camera_info before estimating ArUco pose")
                self._last_warn_time = now
            return

        try:
            image = _image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = _detect_markers(gray, self._dictionary, self._parameters)
        if ids is None:
            return

        ids_flat = ids.flatten()
        matches = np.where(ids_flat == self._marker_id)[0]
        if len(matches) == 0:
            return

        marker_corners = [corners[int(matches[0])]]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners,
            self._marker_size,
            self._camera_matrix,
            self._dist_coeffs,
        )

        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        rotation, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(rotation)
        frame_id = self._camera_frame or msg.header.frame_id

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pose_pub.publish(pose)

        if self._publish_tf:
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = self._marker_frame
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation = pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
