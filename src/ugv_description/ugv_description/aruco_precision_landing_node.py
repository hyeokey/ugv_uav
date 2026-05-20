import math
import time
from typing import Optional

import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


_PX4_FORCE_DISARM_MAGIC = 21196.0


def _nan_array(size: int):
    return [float("nan")] * size


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _yaw_from_px4_quat_wxyz(q) -> Optional[float]:
    try:
        w, x, y, z = [float(v) for v in q]
    except (TypeError, ValueError):
        return None

    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-6:
        return None

    w /= norm
    x /= norm
    y /= norm
    z /= norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ArucoPrecisionLandingNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_precision_landing_node")

        self.declare_parameter("px4_ns", "/px4_0")
        self.declare_parameter("aruco_pose_topic", "/drone/aruco/pose")
        self.declare_parameter("kp_xy", 0.45)
        self.declare_parameter("max_xy_speed", 0.8)
        self.declare_parameter("descent_rate", 0.25)
        self.declare_parameter("center_tolerance", 0.12)
        self.declare_parameter("target_depth", 0.45)
        self.declare_parameter("marker_timeout", 0.5)
        self.declare_parameter("camera_x_to_body_right", 1.0)
        self.declare_parameter("camera_y_to_body_forward", -1.0)
        self.declare_parameter("auto_offboard", False)
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("command_land_when_close", False)
        self.declare_parameter("auto_disarm_when_landed", True)
        self.declare_parameter("force_disarm_when_close", False)
        self.declare_parameter("force_disarm_delay", 1.0)
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        px4_ns = self.get_parameter("px4_ns").value.rstrip("/")
        self._kp_xy = float(self.get_parameter("kp_xy").value)
        self._max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self._descent_rate = abs(float(self.get_parameter("descent_rate").value))
        self._center_tolerance = abs(float(self.get_parameter("center_tolerance").value))
        self._target_depth = abs(float(self.get_parameter("target_depth").value))
        self._marker_timeout = abs(float(self.get_parameter("marker_timeout").value))
        self._camera_x_to_body_right = float(self.get_parameter("camera_x_to_body_right").value)
        self._camera_y_to_body_forward = float(self.get_parameter("camera_y_to_body_forward").value)
        self._auto_offboard = _as_bool(self.get_parameter("auto_offboard").value)
        self._auto_arm = _as_bool(self.get_parameter("auto_arm").value)
        self._command_land_when_close = _as_bool(self.get_parameter("command_land_when_close").value)
        self._auto_disarm_when_landed = _as_bool(self.get_parameter("auto_disarm_when_landed").value)
        self._force_disarm_when_close = _as_bool(self.get_parameter("force_disarm_when_close").value)
        self._force_disarm_delay = abs(float(self.get_parameter("force_disarm_delay").value))
        self._target_system = int(self.get_parameter("target_system").value)
        self._target_component = int(self.get_parameter("target_component").value)
        self._source_system = int(self.get_parameter("source_system").value)
        self._source_component = int(self.get_parameter("source_component").value)

        try:
            from px4_msgs.msg import (
                OffboardControlMode,
                TrajectorySetpoint,
                VehicleCommandAck,
                VehicleCommand,
                VehicleLandDetected,
                VehicleOdometry,
            )
        except ImportError as exc:
            raise RuntimeError(
                "px4_msgs is not sourced. Source the workspace that provides px4_msgs before running this node, "
                "for example: source ~/drone_space/ros2_ws/install/setup.bash"
            ) from exc

        self._OffboardControlMode = OffboardControlMode
        self._TrajectorySetpoint = TrajectorySetpoint
        self._VehicleCommandAck = VehicleCommandAck
        self._VehicleCommand = VehicleCommand
        self._VehicleLandDetected = VehicleLandDetected
        self._land_command_sent = False
        self._disarm_command_sent = False
        self._mode_command_sent = False
        self._arm_command_sent = False
        self._setpoint_count = 0
        self._pose_rx_count = 0
        self._close_since: Optional[float] = None
        self._last_pose: Optional[PoseStamped] = None
        self._last_pose_monotonic: Optional[float] = None
        self._yaw_ned: Optional[float] = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        aruco_pose_topic = self.get_parameter("aruco_pose_topic").value
        self.create_subscription(PoseStamped, aruco_pose_topic, self._on_aruco_pose, 10)
        self.create_subscription(VehicleOdometry, f"{px4_ns}/fmu/out/vehicle_odometry", self._on_odometry, qos)
        self.create_subscription(
            VehicleLandDetected,
            f"{px4_ns}/fmu/out/vehicle_land_detected",
            self._on_land_detected,
            qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            f"{px4_ns}/fmu/out/vehicle_command_ack",
            self._on_vehicle_command_ack,
            qos,
        )

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, f"{px4_ns}/fmu/in/offboard_control_mode", qos)
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, f"{px4_ns}/fmu/in/trajectory_setpoint", qos)
        self._command_pub = self.create_publisher(VehicleCommand, f"{px4_ns}/fmu/in/vehicle_command", qos)
        self._debug_pub = self.create_publisher(TwistStamped, "/drone/precland/velocity_cmd", 10)

        self.create_timer(0.05, self._on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"Precision landing using {aruco_pose_topic}; publishing velocity setpoints to {px4_ns}"
        )

    def _timestamp_us(self) -> int:
        return int(time.monotonic_ns() / 1000)

    def _on_aruco_pose(self, msg: PoseStamped) -> None:
        self._last_pose = msg
        self._last_pose_monotonic = time.monotonic()
        self._pose_rx_count += 1
        if self._pose_rx_count == 1:
            self.get_logger().info(
                f"Received first ArUco pose on {self.get_parameter('aruco_pose_topic').value}: "
                f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
            )

    def _on_odometry(self, msg) -> None:
        yaw = _yaw_from_px4_quat_wxyz(msg.q)
        if yaw is not None:
            self._yaw_ned = yaw

    def _on_land_detected(self, msg) -> None:
        if self._auto_disarm_when_landed and msg.landed and not self._disarm_command_sent:
            self._publish_disarm("PX4 land detector reports landed")

    def _on_vehicle_command_ack(self, msg) -> None:
        tracked_commands = (
            self._VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            self._VehicleCommand.VEHICLE_CMD_NAV_LAND,
            self._VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        )
        if msg.command in tracked_commands:
            self.get_logger().info(f"PX4 command ack: command={msg.command} result={msg.result}")

    def _marker_is_fresh(self) -> bool:
        if self._last_pose is None:
            return False

        if self._last_pose_monotonic is None:
            return False

        age = time.monotonic() - self._last_pose_monotonic
        return age <= self._marker_timeout

    def _publish_offboard_control_mode(self) -> None:
        msg = self._OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self._offboard_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = self._VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = self._target_system
        msg.target_component = self._target_component
        msg.source_system = self._source_system
        msg.source_component = self._source_component
        msg.from_external = True
        self._command_pub.publish(msg)

    def _publish_disarm(self, reason: str, force: bool = False) -> None:
        self._publish_vehicle_command(
            self._VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            float(self._VehicleCommand.ARMING_ACTION_DISARM),
            _PX4_FORCE_DISARM_MAGIC if force else 0.0,
        )
        self._disarm_command_sent = True
        if force:
            self.get_logger().warn(f"Requested PX4 FORCE disarm: {reason}")
        else:
            self.get_logger().info(f"Requested PX4 disarm: {reason}")

    def _maybe_publish_mode_commands(self) -> None:
        if self._setpoint_count < 20:
            return

        if self._auto_offboard and not self._mode_command_sent:
            self._publish_vehicle_command(self._VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._mode_command_sent = True
            self.get_logger().info("Requested PX4 Offboard mode")

        if self._auto_arm and not self._arm_command_sent:
            self._publish_vehicle_command(
                self._VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                float(self._VehicleCommand.ARMING_ACTION_ARM),
            )
            self._arm_command_sent = True
            self.get_logger().info("Requested PX4 arm")

    def _body_frd_to_ned_velocity(self, forward: float, right: float, down: float):
        yaw = self._yaw_ned if self._yaw_ned is not None else 0.0
        north = math.cos(yaw) * forward - math.sin(yaw) * right
        east = math.sin(yaw) * forward + math.cos(yaw) * right
        return [float(north), float(east), float(down)]

    def _compute_velocity_ned(self):
        if not self._marker_is_fresh():
            return [0.0, 0.0, 0.0], False, float("nan")

        pose = self._last_pose.pose.position
        forward_error = self._camera_y_to_body_forward * float(pose.y)
        right_error = self._camera_x_to_body_right * float(pose.x)
        depth = float(pose.z)

        forward = _clamp(self._kp_xy * forward_error, -self._max_xy_speed, self._max_xy_speed)
        right = _clamp(self._kp_xy * right_error, -self._max_xy_speed, self._max_xy_speed)
        centered = math.hypot(forward_error, right_error) <= self._center_tolerance
        down = self._descent_rate if centered and depth > self._target_depth else 0.0

        close = centered and depth <= self._target_depth

        if self._command_land_when_close and close and not self._land_command_sent:
            self._publish_vehicle_command(self._VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._land_command_sent = True
            self.get_logger().info("Requested PX4 Land: marker centered and target depth reached")

        if close:
            if self._close_since is None:
                self._close_since = time.monotonic()
            elif (
                self._force_disarm_when_close
                and not self._disarm_command_sent
                and (time.monotonic() - self._close_since) >= self._force_disarm_delay
            ):
                self._publish_disarm("marker target depth held", force=True)
        else:
            self._close_since = None

        return self._body_frd_to_ned_velocity(forward, right, down), centered, depth

    def _publish_trajectory_setpoint(self, velocity_ned) -> None:
        msg = self._TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position = _nan_array(3)
        msg.velocity = [float(v) for v in velocity_ned]
        msg.acceleration = _nan_array(3)
        msg.jerk = _nan_array(3)
        msg.yaw = float("nan")
        msg.yawspeed = float("nan")
        self._trajectory_pub.publish(msg)

        debug = TwistStamped()
        debug.header.stamp = self.get_clock().now().to_msg()
        debug.header.frame_id = "ned"
        debug.twist.linear.x = float(velocity_ned[0])
        debug.twist.linear.y = float(velocity_ned[1])
        debug.twist.linear.z = float(velocity_ned[2])
        self._debug_pub.publish(debug)

    def _on_timer(self) -> None:
        self._publish_offboard_control_mode()
        velocity_ned, centered, depth = self._compute_velocity_ned()
        self._publish_trajectory_setpoint(velocity_ned)
        self._setpoint_count += 1
        self._maybe_publish_mode_commands()

        if self._setpoint_count % 20 == 0:
            if self._last_pose is None:
                self.get_logger().info("Waiting for ArUco pose")
            else:
                pose_age = (
                    time.monotonic() - self._last_pose_monotonic
                    if self._last_pose_monotonic is not None else float("nan")
                )
                self.get_logger().info(
                    f"precland vel_ned=[{velocity_ned[0]:.2f}, {velocity_ned[1]:.2f}, {velocity_ned[2]:.2f}] "
                    f"centered={centered} depth={depth:.2f} pose_age={pose_age:.2f}s"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoPrecisionLandingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
