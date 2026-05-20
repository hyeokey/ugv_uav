import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


def _nan_array(size: int):
    return [float("nan")] * size


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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


def _geodetic_offset_ned(lat_from: float, lon_from: float, lat_to: float, lon_to: float) -> Tuple[float, float]:
    earth_radius_m = 6378137.0
    lat1 = math.radians(lat_from)
    lat2 = math.radians(lat_to)
    d_lat = lat2 - lat1
    d_lon = math.radians(lon_to - lon_from)
    mean_lat = 0.5 * (lat1 + lat2)
    north = earth_radius_m * d_lat
    east = earth_radius_m * math.cos(mean_lat) * d_lon
    return north, east


class LidarLocalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_local_planner_node")

        self.declare_parameter("px4_ns", "/px4_1")
        self.declare_parameter("scan_topic", "/ugv/lidar/scan")
        self.declare_parameter("marker_frame", "odom")
        self.declare_parameter("goal_source", "manual")
        self.declare_parameter("manual_goal_frame", "relative")
        self.declare_parameter("heading_source", "course")
        self.declare_parameter("goal_north", 0.0)
        self.declare_parameter("goal_east", 0.0)
        self.declare_parameter("goal_tolerance", 1.0)
        self.declare_parameter("local_target_distance", 1.5)
        self.declare_parameter("desired_speed", 0.8)
        self.declare_parameter("max_speed", 1.2)
        self.declare_parameter("slowdown_distance", 2.0)
        self.declare_parameter("min_turn_speed_scale", 0.25)
        self.declare_parameter("max_steering_angle", 0.75)
        self.declare_parameter("turn_slowdown_angle", 1.2)
        self.declare_parameter("obstacle_range", 1.5)
        self.declare_parameter("stop_range", 0.45)
        self.declare_parameter("avoid_gain", 1.4)
        self.declare_parameter("front_fov_deg", 180.0)
        self.declare_parameter("lidar_yaw_offset", 0.0)
        self.declare_parameter("avoidance_corridor_deg", 70.0)
        self.declare_parameter("avoidance_side_bias", 3.2)
        self.declare_parameter("scan_timeout", 0.6)
        self.declare_parameter("course_timeout", 1.0)
        self.declare_parameter("course_min_distance", 0.02)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("publish_trajectory_setpoint", False)
        self.declare_parameter("publish_rover_setpoints", True)
        self.declare_parameter("rover_yaw_offset", 0.0)
        self.declare_parameter("rover_steering_sign", -1.0)
        self.declare_parameter("auto_offboard", False)
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        self._px4_ns = str(self.get_parameter("px4_ns").value).rstrip("/")
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._marker_frame = str(self.get_parameter("marker_frame").value)
        self._goal_source = str(self.get_parameter("goal_source").value).strip().lower()
        self._manual_goal_frame = str(self.get_parameter("manual_goal_frame").value).strip().lower()
        self._heading_source = str(self.get_parameter("heading_source").value).strip().lower()
        self._goal_north = float(self.get_parameter("goal_north").value)
        self._goal_east = float(self.get_parameter("goal_east").value)
        self._goal_tolerance = abs(float(self.get_parameter("goal_tolerance").value))
        self._local_target_distance = abs(float(self.get_parameter("local_target_distance").value))
        self._desired_speed = abs(float(self.get_parameter("desired_speed").value))
        self._max_speed = abs(float(self.get_parameter("max_speed").value))
        self._slowdown_distance = abs(float(self.get_parameter("slowdown_distance").value))
        self._min_turn_speed_scale = _clamp(
            abs(float(self.get_parameter("min_turn_speed_scale").value)),
            0.0,
            1.0,
        )
        self._max_steering_angle = abs(float(self.get_parameter("max_steering_angle").value))
        self._turn_slowdown_angle = abs(float(self.get_parameter("turn_slowdown_angle").value))
        self._obstacle_range = abs(float(self.get_parameter("obstacle_range").value))
        self._stop_range = abs(float(self.get_parameter("stop_range").value))
        self._avoid_gain = float(self.get_parameter("avoid_gain").value)
        self._front_fov = math.radians(abs(float(self.get_parameter("front_fov_deg").value)))
        self._lidar_yaw_offset = float(self.get_parameter("lidar_yaw_offset").value)
        self._avoidance_corridor = math.radians(abs(float(self.get_parameter("avoidance_corridor_deg").value)))
        self._avoidance_side_bias = abs(float(self.get_parameter("avoidance_side_bias").value))
        self._scan_timeout = abs(float(self.get_parameter("scan_timeout").value))
        self._course_timeout = abs(float(self.get_parameter("course_timeout").value))
        self._course_min_distance = abs(float(self.get_parameter("course_min_distance").value))
        self._allow_reverse = _as_bool(self.get_parameter("allow_reverse").value)
        self._publish_trajectory = _as_bool(self.get_parameter("publish_trajectory_setpoint").value)
        self._publish_rover = _as_bool(self.get_parameter("publish_rover_setpoints").value)
        self._rover_yaw_offset = float(self.get_parameter("rover_yaw_offset").value)
        self._rover_steering_sign = float(self.get_parameter("rover_steering_sign").value)
        self._auto_offboard = _as_bool(self.get_parameter("auto_offboard").value)
        self._auto_arm = _as_bool(self.get_parameter("auto_arm").value)
        self._target_system = int(self.get_parameter("target_system").value)
        self._target_component = int(self.get_parameter("target_component").value)
        self._source_system = int(self.get_parameter("source_system").value)
        self._source_component = int(self.get_parameter("source_component").value)

        try:
            from px4_msgs.msg import (
                NavigatorMissionItem,
                OffboardControlMode,
                RoverSpeedSetpoint,
                RoverSteeringSetpoint,
                TrajectorySetpoint,
                VehicleCommand,
                VehicleCommandAck,
                VehicleGlobalPosition,
                VehicleOdometry,
            )
        except ImportError as exc:
            raise RuntimeError(
                "px4_msgs is not sourced. Source the workspace that provides px4_msgs before running this node, "
                "for example: source ~/drone_space/ros2_ws/install/setup.bash"
            ) from exc

        self._NavigatorMissionItem = NavigatorMissionItem
        self._OffboardControlMode = OffboardControlMode
        self._RoverSpeedSetpoint = RoverSpeedSetpoint
        self._RoverSteeringSetpoint = RoverSteeringSetpoint
        self._TrajectorySetpoint = TrajectorySetpoint
        self._VehicleCommand = VehicleCommand
        self._VehicleCommandAck = VehicleCommandAck
        self._VehicleGlobalPosition = VehicleGlobalPosition
        self._VehicleOdometry = VehicleOdometry

        self._position_ned: Optional[Tuple[float, float]] = None
        self._previous_position_ned: Optional[Tuple[float, float]] = None
        self._course_reference_position_ned: Optional[Tuple[float, float]] = None
        self._manual_origin_ned: Optional[Tuple[float, float]] = None
        self._manual_target_ned: Optional[Tuple[float, float]] = None
        self._yaw_ned: Optional[float] = None
        self._course_yaw_ned: Optional[float] = None
        self._last_course_monotonic: Optional[float] = None
        self._global_position = None
        self._mission_item = None
        self._global_position_logged = False
        self._mission_sequence_logged: Optional[int] = None
        self._goal_debug_text = "target=[nan, nan]"
        self._goal_marker_ned: Optional[Tuple[float, float]] = None
        self._local_target_marker_ned: Optional[Tuple[float, float]] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_scan_monotonic: Optional[float] = None
        self._setpoint_count = 0
        self._mode_command_sent = False
        self._arm_command_sent = False
        self._last_speed_body_x = 0.0
        self._last_steering = 0.0
        self._nearest_front = float("nan")
        self._left_clearance = float("nan")
        self._right_clearance = float("nan")
        self._avoidance_side = 0.0

        px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            VehicleOdometry,
            f"{self._px4_ns}/fmu/out/vehicle_odometry",
            self._on_odometry,
            px4_qos,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            f"{self._px4_ns}/fmu/out/vehicle_global_position",
            self._on_global_position,
            px4_qos,
        )
        self.create_subscription(
            NavigatorMissionItem,
            f"{self._px4_ns}/fmu/out/navigator_mission_item",
            self._on_mission_item,
            px4_qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            f"{self._px4_ns}/fmu/out/vehicle_command_ack",
            self._on_vehicle_command_ack,
            px4_qos,
        )

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, f"{self._px4_ns}/fmu/in/offboard_control_mode", px4_qos)
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint, f"{self._px4_ns}/fmu/in/trajectory_setpoint", px4_qos)
        self._rover_speed_pub = self.create_publisher(
            RoverSpeedSetpoint, f"{self._px4_ns}/fmu/in/rover_speed_setpoint", px4_qos)
        self._rover_steering_pub = self.create_publisher(
            RoverSteeringSetpoint, f"{self._px4_ns}/fmu/in/rover_steering_setpoint", px4_qos)
        self._command_pub = self.create_publisher(
            VehicleCommand, f"{self._px4_ns}/fmu/in/vehicle_command", px4_qos)
        self._debug_pub = self.create_publisher(TwistStamped, "/ugv/local_planner/velocity_cmd", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/ugv/local_planner/markers", 10)
        self._global_goal_marker_pub = self.create_publisher(Marker, "/ugv/local_planner/global_goal_marker", 10)
        self._local_target_marker_pub = self.create_publisher(Marker, "/ugv/local_planner/local_target_marker", 10)

        self.create_timer(0.05, self._on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"LiDAR local planner using {self._scan_topic}; px4_ns={self._px4_ns}; "
            f"target_system={self._target_system}; goal_source={self._goal_source}; "
            f"manual_goal_frame={self._manual_goal_frame}; heading_source={self._heading_source}; "
            f"publish_trajectory={self._publish_trajectory}; publish_rover={self._publish_rover}"
        )

    def _timestamp_us(self) -> int:
        return int(time.monotonic_ns() / 1000)

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._last_scan_monotonic = time.monotonic()

    def _on_odometry(self, msg) -> None:
        try:
            north = float(msg.position[0])
            east = float(msg.position[1])
        except (TypeError, ValueError, IndexError):
            return

        if math.isfinite(north) and math.isfinite(east):
            if self._course_reference_position_ned is None:
                self._course_reference_position_ned = (north, east)
            else:
                prev_north, prev_east = self._course_reference_position_ned
                delta_north = north - prev_north
                delta_east = east - prev_east
                if math.hypot(delta_north, delta_east) >= self._course_min_distance:
                    self._course_yaw_ned = math.atan2(delta_east, delta_north)
                    self._last_course_monotonic = time.monotonic()
                    self._previous_position_ned = self._position_ned
                    self._course_reference_position_ned = (north, east)
            self._position_ned = (north, east)
            if self._manual_origin_ned is None:
                self._manual_origin_ned = (north, east)
                if self._manual_goal_frame == "relative":
                    self._manual_target_ned = (north + self._goal_north, east + self._goal_east)
                    self.get_logger().info(
                        f"Manual relative goal origin north={north:.2f}, east={east:.2f}; "
                        f"target north={self._manual_target_ned[0]:.2f}, east={self._manual_target_ned[1]:.2f}"
                    )
                else:
                    self._manual_target_ned = (self._goal_north, self._goal_east)
                    self.get_logger().info(
                        f"Manual absolute goal target north={self._manual_target_ned[0]:.2f}, "
                        f"east={self._manual_target_ned[1]:.2f}"
                    )

        yaw = _yaw_from_px4_quat_wxyz(msg.q)
        if yaw is not None:
            self._yaw_ned = yaw

    def _heading_yaw_ned(self) -> Optional[float]:
        if self._heading_source == "odom":
            return self._yaw_ned

        course_is_fresh = (
            self._course_yaw_ned is not None
            and self._last_course_monotonic is not None
            and (time.monotonic() - self._last_course_monotonic) <= self._course_timeout
        )
        if self._heading_source == "course":
            return self._course_yaw_ned if course_is_fresh else self._yaw_ned

        if self._heading_source == "hybrid":
            return self._course_yaw_ned if course_is_fresh else self._yaw_ned

        self.get_logger().warn(
            f"Unknown heading_source={self._heading_source}; expected course, hybrid, or odom",
            throttle_duration_sec=2.0,
        )
        return self._course_yaw_ned if course_is_fresh else self._yaw_ned

    def _on_global_position(self, msg) -> None:
        if msg.lat_lon_valid and math.isfinite(msg.lat) and math.isfinite(msg.lon):
            self._global_position = msg
            if not self._global_position_logged:
                self._global_position_logged = True
                self.get_logger().info(
                    f"Received PX4 global position: lat={msg.lat:.7f}, lon={msg.lon:.7f}, alt={msg.alt:.2f}"
                )

    def _on_mission_item(self, msg) -> None:
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self._mission_item = msg
            if self._mission_sequence_logged != int(msg.sequence_current):
                self._mission_sequence_logged = int(msg.sequence_current)
                self.get_logger().info(
                    f"Received QGC mission item: seq={msg.sequence_current} nav_cmd={msg.nav_cmd} "
                    f"lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, acceptance={msg.acceptance_radius:.2f}"
                )

    def _on_vehicle_command_ack(self, msg) -> None:
        tracked_commands = (
            self._VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            self._VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        )
        if msg.command in tracked_commands:
            self.get_logger().info(f"PX4 command ack: command={msg.command} result={msg.result}")

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

    def _goal_vector_ned(self) -> Optional[Tuple[float, float, float, str]]:
        if self._goal_source == "manual":
            if self._position_ned is None:
                return None
            north, east = self._position_ned
            if self._manual_goal_frame == "relative":
                if self._manual_origin_ned is None:
                    return None
                if self._manual_target_ned is None:
                    origin_north, origin_east = self._manual_origin_ned
                    self._manual_target_ned = (origin_north + self._goal_north, origin_east + self._goal_east)
                target_north, target_east = self._manual_target_ned
            else:
                target_north = self._goal_north
                target_east = self._goal_east
                self._manual_target_ned = (target_north, target_east)

            goal_north = target_north - north
            goal_east = target_east - east
            self._goal_marker_ned = (target_north, target_east)
            self._goal_debug_text = f"target=[{target_north:.2f}, {target_east:.2f}]"
            return goal_north, goal_east, math.hypot(goal_north, goal_east), "manual"

        if self._goal_source == "mission":
            if self._global_position is None or self._mission_item is None:
                return None
            goal_north, goal_east = _geodetic_offset_ned(
                self._global_position.lat,
                self._global_position.lon,
                self._mission_item.latitude,
                self._mission_item.longitude,
            )
            if self._position_ned is not None:
                self._goal_marker_ned = (
                    self._position_ned[0] + goal_north,
                    self._position_ned[1] + goal_east,
                )
            self._goal_debug_text = (
                f"mission_vec=[{goal_north:.2f}, {goal_east:.2f}] "
                f"seq={self._mission_item.sequence_current}"
            )
            return goal_north, goal_east, math.hypot(goal_north, goal_east), "mission"

        self.get_logger().warn(f"Unknown goal_source={self._goal_source}; expected manual or mission", throttle_duration_sec=2.0)
        return None

    def _scan_is_fresh(self) -> bool:
        if self._last_scan is None or self._last_scan_monotonic is None:
            return False
        return (time.monotonic() - self._last_scan_monotonic) <= self._scan_timeout

    def _lidar_repulsion_body(self) -> Tuple[float, float, float, float, float, bool]:
        if not self._scan_is_fresh():
            return 0.0, 0.0, float("nan"), float("nan"), float("nan"), False

        scan = self._last_scan
        half_fov = 0.5 * self._front_fov
        half_corridor = 0.5 * self._avoidance_corridor
        angle = scan.angle_min
        repulse_forward = 0.0
        repulse_left = 0.0
        nearest_front = float("inf")
        left_clearance = float("inf")
        right_clearance = float("inf")

        for range_value in scan.ranges:
            body_angle = _wrap_pi(angle + self._lidar_yaw_offset)
            if (
                math.isfinite(range_value)
                and scan.range_min <= range_value <= scan.range_max
                and range_value <= self._obstacle_range
                and abs(body_angle) <= half_fov
            ):
                x = range_value * math.cos(body_angle)
                y_left = range_value * math.sin(body_angle)
                if x > 0.0 and abs(body_angle) <= half_corridor:
                    nearest_front = min(nearest_front, range_value)
                elif body_angle > half_corridor:
                    left_clearance = min(left_clearance, range_value)
                elif body_angle < -half_corridor:
                    right_clearance = min(right_clearance, range_value)

                weight = ((self._obstacle_range - range_value) / self._obstacle_range) ** 2
                repulse_forward += -x / max(range_value, 1e-3) * weight
                repulse_left += -y_left / max(range_value, 1e-3) * weight

            angle += scan.angle_increment

        if not math.isfinite(nearest_front):
            nearest_front = float("nan")
        if not math.isfinite(left_clearance):
            left_clearance = float("nan")
        if not math.isfinite(right_clearance):
            right_clearance = float("nan")

        return repulse_forward, repulse_left, nearest_front, left_clearance, right_clearance, True

    def _compute_velocity_ned(self) -> Tuple[float, float, float, bool, float, str]:
        goal = self._goal_vector_ned()
        heading_yaw = self._heading_yaw_ned()
        if goal is None or heading_yaw is None:
            return 0.0, 0.0, float("nan"), False, float("nan"), "waiting"

        goal_north, goal_east, goal_distance, goal_label = goal
        if goal_distance <= self._goal_tolerance:
            return 0.0, 0.0, heading_yaw, True, goal_distance, goal_label

        goal_unit_north = goal_north / max(goal_distance, 1e-6)
        goal_unit_east = goal_east / max(goal_distance, 1e-6)

        yaw = heading_yaw
        goal_forward = math.cos(yaw) * goal_unit_north + math.sin(yaw) * goal_unit_east
        goal_left = math.sin(yaw) * goal_unit_north - math.cos(yaw) * goal_unit_east

        repulse_forward, repulse_left, nearest_front, left_clearance, right_clearance, scan_ok = self._lidar_repulsion_body()
        if not scan_ok:
            repulse_forward = 0.0
            repulse_left = 0.0
            left_clearance = float("nan")
            right_clearance = float("nan")

        self._nearest_front = nearest_front
        self._left_clearance = left_clearance
        self._right_clearance = right_clearance
        self._avoidance_side = 0.0
        self._local_target_marker_ned = None

        desired_forward = goal_forward + self._avoid_gain * repulse_forward
        desired_left = goal_left + self._avoid_gain * repulse_left

        obstacle_scale = 1.0
        if math.isfinite(nearest_front) and nearest_front <= self._obstacle_range:
            left_for_compare = left_clearance if math.isfinite(left_clearance) else self._obstacle_range
            right_for_compare = right_clearance if math.isfinite(right_clearance) else self._obstacle_range
            if abs(left_for_compare - right_for_compare) < 0.05:
                side = 1.0 if goal_left >= 0.0 else -1.0
            else:
                side = 1.0 if left_for_compare > right_for_compare else -1.0

            closeness = _clamp(
                (self._obstacle_range - nearest_front) / max(self._obstacle_range - self._stop_range, 1e-3),
                0.0,
                1.0,
            )
            desired_forward = max(0.0, desired_forward * (1.0 - 0.85 * closeness))
            desired_left += side * self._avoidance_side_bias * (0.35 + closeness)
            obstacle_scale = _clamp(
                (nearest_front - self._stop_range) / max(self._obstacle_range - self._stop_range, 1e-3),
                0.25,
                1.0,
            )
            self._avoidance_side = side

        if math.isfinite(nearest_front) and nearest_front <= self._stop_range:
            desired_forward = 0.0

        if not self._allow_reverse and desired_forward < 0.0:
            desired_forward = 0.0
            if abs(desired_left) < 1e-3:
                desired_left = 1.0 if goal_left >= 0.0 else -1.0

        norm = math.hypot(desired_forward, desired_left)
        if norm <= 1e-6:
            return 0.0, 0.0, yaw, False, goal_distance, goal_label

        desired_forward /= norm
        desired_left /= norm

        if self._position_ned is not None:
            lookahead = min(self._local_target_distance, max(goal_distance, 0.0))
            local_target_north = self._position_ned[0] + lookahead * (
                math.cos(yaw) * desired_forward + math.sin(yaw) * desired_left
            )
            local_target_east = self._position_ned[1] + lookahead * (
                math.sin(yaw) * desired_forward - math.cos(yaw) * desired_left
            )
            self._local_target_marker_ned = (local_target_north, local_target_east)

        heading_error = math.atan2(desired_left, desired_forward)
        distance_scale = _clamp(goal_distance / max(self._slowdown_distance, 1e-3), 0.25, 1.0)
        turn_scale = 1.0 - 0.25 * _clamp(abs(heading_error) / max(self._turn_slowdown_angle, 1e-3), 0.0, 1.0)
        speed = min(self._desired_speed, self._max_speed) * distance_scale * turn_scale * obstacle_scale

        vel_north = speed * (math.cos(yaw) * desired_forward + math.sin(yaw) * desired_left)
        vel_east = speed * (math.sin(yaw) * desired_forward - math.cos(yaw) * desired_left)
        yaw_sp = _wrap_pi(math.atan2(vel_east, vel_north)) if speed > 0.01 else yaw

        return vel_north, vel_east, yaw_sp, False, goal_distance, goal_label

    def _publish_trajectory_setpoint(self, vel_north: float, vel_east: float, yaw_sp: float) -> None:
        msg = self._TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position = _nan_array(3)
        msg.velocity = [float(vel_north), float(vel_east), 0.0]
        msg.acceleration = _nan_array(3)
        msg.jerk = _nan_array(3)
        msg.yaw = float(yaw_sp) if math.isfinite(yaw_sp) else float("nan")
        msg.yawspeed = float("nan")
        self._trajectory_pub.publish(msg)

        debug = TwistStamped()
        debug.header.stamp = self.get_clock().now().to_msg()
        debug.header.frame_id = "ned"
        debug.twist.linear.x = float(vel_north)
        debug.twist.linear.y = float(vel_east)
        debug.twist.angular.z = msg.yaw
        self._debug_pub.publish(debug)

    def _publish_rover_setpoints(self, vel_north: float, vel_east: float, yaw_sp: float) -> None:
        heading_yaw = self._heading_yaw_ned()
        if heading_yaw is None:
            return

        speed = math.hypot(vel_north, vel_east)
        if speed <= 0.01 or not math.isfinite(yaw_sp):
            speed_body_x = 0.0
            steering = 0.0
        else:
            heading_error = _wrap_pi(yaw_sp - (heading_yaw + self._rover_yaw_offset))
            speed_body_x = max(speed, self._desired_speed * self._min_turn_speed_scale)
            steering = _clamp(
                self._rover_steering_sign * heading_error / max(self._max_steering_angle, 1e-3),
                -1.0,
                1.0,
            )

        speed_msg = self._RoverSpeedSetpoint()
        speed_msg.timestamp = self._timestamp_us()
        speed_msg.speed_body_x = float(speed_body_x)
        speed_msg.speed_body_y = float("nan")
        self._rover_speed_pub.publish(speed_msg)

        steering_msg = self._RoverSteeringSetpoint()
        steering_msg.timestamp = self._timestamp_us()
        steering_msg.normalized_steering_setpoint = float(steering)
        self._rover_steering_pub.publish(steering_msg)
        self._last_speed_body_x = float(speed_body_x)
        self._last_steering = float(steering)

    def _make_sphere_marker(self, marker_id: int, name: str, ned_xy, color, scale: float) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._marker_frame
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        north, east = ned_xy
        marker.pose.position.x = float(east)
        marker.pose.position.y = float(north)
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        return marker

    def _make_delete_marker(self, marker_id: int, name: str) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._marker_frame
        marker.ns = name
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        if self._goal_marker_ned is not None:
            global_goal_marker = self._make_sphere_marker(
                0, "global_goal", self._goal_marker_ned, (1.0, 0.0, 0.0, 0.9), 0.8)
            markers.markers.append(global_goal_marker)
            self._global_goal_marker_pub.publish(global_goal_marker)
        else:
            delete_global_goal = self._make_delete_marker(0, "global_goal")
            markers.markers.append(delete_global_goal)
            self._global_goal_marker_pub.publish(delete_global_goal)

        if self._local_target_marker_ned is not None:
            local_target_marker = self._make_sphere_marker(
                1,
                "local_target",
                self._local_target_marker_ned,
                (0.0, 1.0, 0.1, 0.95),
                0.9,
            )
            markers.markers.append(local_target_marker)
            self._local_target_marker_pub.publish(local_target_marker)
        else:
            delete_local_target = self._make_delete_marker(1, "local_target")
            markers.markers.append(delete_local_target)
            self._local_target_marker_pub.publish(delete_local_target)

        self._marker_pub.publish(markers)

    def _on_timer(self) -> None:
        self._publish_offboard_control_mode()
        vel_north, vel_east, yaw_sp, reached, goal_distance, goal_label = self._compute_velocity_ned()
        if self._publish_trajectory:
            self._publish_trajectory_setpoint(vel_north, vel_east, yaw_sp)
        if self._publish_rover:
            self._publish_rover_setpoints(vel_north, vel_east, yaw_sp)
        self._publish_markers()
        self._setpoint_count += 1
        self._maybe_publish_mode_commands()

        if self._setpoint_count % 20 == 0:
            scan_age = (
                time.monotonic() - self._last_scan_monotonic
                if self._last_scan_monotonic is not None else float("nan")
            )
            position_text = "pos=[nan, nan]"
            target_text = self._goal_debug_text
            if self._position_ned is not None:
                position_text = f"pos=[{self._position_ned[0]:.2f}, {self._position_ned[1]:.2f}]"
            if self._goal_source == "manual" and self._manual_target_ned is not None:
                target_text = f"target=[{self._manual_target_ned[0]:.2f}, {self._manual_target_ned[1]:.2f}]"
            local_target_text = "local_target=[nan, nan]"
            if self._local_target_marker_ned is not None:
                local_target_text = (
                    f"local_target=[{self._local_target_marker_ned[0]:.2f}, "
                    f"{self._local_target_marker_ned[1]:.2f}]"
                )
            heading_yaw = self._heading_yaw_ned()
            self.get_logger().info(
                f"local_planner goal={goal_label} dist={goal_distance:.2f}m reached={reached} "
                f"{position_text} {target_text} {local_target_text} "
                f"yaw={heading_yaw if heading_yaw is not None else float('nan'):.2f} "
                f"yaw_odom={self._yaw_ned if self._yaw_ned is not None else float('nan'):.2f} "
                f"yaw_course={self._course_yaw_ned if self._course_yaw_ned is not None else float('nan'):.2f} "
                f"vel_ned=[{vel_north:.2f}, {vel_east:.2f}, 0.00] yaw_sp={yaw_sp:.2f} "
                f"rover=[speed={self._last_speed_body_x:.2f}, steer={self._last_steering:.2f}] "
                f"obstacle=[front={self._nearest_front:.2f}, left={self._left_clearance:.2f}, "
                f"right={self._right_clearance:.2f}, side={self._avoidance_side:.0f}] "
                f"scan_age={scan_age:.2f}s"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarLocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
