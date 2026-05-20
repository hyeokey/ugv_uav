from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/ugv/lidar/scan"),
        DeclareLaunchArgument("marker_topic", default_value="/ugv/lidar/debug_markers"),
        DeclareLaunchArgument("assumed_front_deg", default_value="0.0"),
        DeclareLaunchArgument("sector_width_deg", default_value="20.0"),
        DeclareLaunchArgument("max_debug_range", default_value="5.0"),
        DeclareLaunchArgument("log_period", default_value="1.0"),
        DeclareLaunchArgument("publish_markers", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="ugv_description",
            executable="lidar_scan_debug_node",
            name="lidar_scan_debug",
            output="screen",
            parameters=[{
                "scan_topic": LaunchConfiguration("scan_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "assumed_front_deg": LaunchConfiguration("assumed_front_deg"),
                "sector_width_deg": LaunchConfiguration("sector_width_deg"),
                "max_debug_range": LaunchConfiguration("max_debug_range"),
                "log_period": LaunchConfiguration("log_period"),
                "publish_markers": LaunchConfiguration("publish_markers"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
