from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("ugv_description"))
    urdf_path = pkg_share / "urdf" / "explorer_rover.urdf"
    rviz_path = pkg_share / "rviz" / "ugv_lidar.rviz"

    robot_description = urdf_path.read_text()

    return LaunchDescription([
        DeclareLaunchArgument("odometry_topic", default_value="/px4_1/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("parent_frame", default_value="odom"),
        DeclareLaunchArgument("child_frame", default_value="base_link"),
        DeclareLaunchArgument("publish_2d", default_value="true"),
        DeclareLaunchArgument("zero_z", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="ugv_robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "frame_prefix": "",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
        Node(
            package="ugv_description",
            executable="px4_odometry_tf_node",
            name="ugv_px4_odometry_tf",
            output="screen",
            parameters=[{
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "parent_frame": LaunchConfiguration("parent_frame"),
                "child_frame": LaunchConfiguration("child_frame"),
                "publish_2d": LaunchConfiguration("publish_2d"),
                "zero_z": LaunchConfiguration("zero_z"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
        Node(
            package="ugv_description",
            executable="laser_scan_to_pointcloud_node",
            name="ugv_laser_scan_to_pointcloud",
            output="screen",
            parameters=[{
                "scan_topic": "/ugv/lidar/scan",
                "points_topic": "/ugv/lidar/points",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_path)],
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
