from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
        DeclareLaunchArgument("px4_ns", default_value="/"),
        DeclareLaunchArgument("aruco_pose_topic", default_value="/drone/aruco/pose"),
        DeclareLaunchArgument("kp_xy", default_value="0.45"),
        DeclareLaunchArgument("max_xy_speed", default_value="0.8"),
        DeclareLaunchArgument("descent_rate", default_value="0.25"),
        DeclareLaunchArgument("center_tolerance", default_value="0.12"),
        DeclareLaunchArgument("target_depth", default_value="0.15"),
        DeclareLaunchArgument("marker_timeout", default_value="0.5"),
        DeclareLaunchArgument("camera_x_to_body_right", default_value="1.0"),
        DeclareLaunchArgument("camera_y_to_body_forward", default_value="-1.0"),
        DeclareLaunchArgument("auto_offboard", default_value="false"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("command_land_when_close", default_value="true"),
        DeclareLaunchArgument("auto_disarm_when_landed", default_value="true"),
        DeclareLaunchArgument("force_disarm_when_close", default_value="false"),
        DeclareLaunchArgument("force_disarm_delay", default_value="1.0"),
        DeclareLaunchArgument("target_system", default_value="1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="ugv_description",
            executable="aruco_precision_landing_node",
            name="aruco_precision_landing",
            output="screen",
            parameters=[{
                "px4_ns": LaunchConfiguration("px4_ns"),
                "aruco_pose_topic": LaunchConfiguration("aruco_pose_topic"),
                "kp_xy": LaunchConfiguration("kp_xy"),
                "max_xy_speed": LaunchConfiguration("max_xy_speed"),
                "descent_rate": LaunchConfiguration("descent_rate"),
                "center_tolerance": LaunchConfiguration("center_tolerance"),
                "target_depth": LaunchConfiguration("target_depth"),
                "marker_timeout": LaunchConfiguration("marker_timeout"),
                "camera_x_to_body_right": LaunchConfiguration("camera_x_to_body_right"),
                "camera_y_to_body_forward": LaunchConfiguration("camera_y_to_body_forward"),
                "auto_offboard": LaunchConfiguration("auto_offboard"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "command_land_when_close": LaunchConfiguration("command_land_when_close"),
                "auto_disarm_when_landed": LaunchConfiguration("auto_disarm_when_landed"),
                "force_disarm_when_close": LaunchConfiguration("force_disarm_when_close"),
                "force_disarm_delay": LaunchConfiguration("force_disarm_delay"),
                "target_system": LaunchConfiguration("target_system"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
