from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
        DeclareLaunchArgument("image_topic", default_value="/drone/down_camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/drone/down_camera/camera_info"),
        DeclareLaunchArgument("pose_topic", default_value="/drone/aruco/pose"),
        DeclareLaunchArgument("marker_id", default_value="0"),
        DeclareLaunchArgument("marker_size", default_value="0.32"),
        DeclareLaunchArgument("dictionary", default_value="DICT_4X4_50"),
        DeclareLaunchArgument("marker_frame", default_value="rover_aruco_marker"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="ugv_description",
            executable="aruco_detector_node",
            name="aruco_detector",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "marker_id": LaunchConfiguration("marker_id"),
                "marker_size": LaunchConfiguration("marker_size"),
                "dictionary": LaunchConfiguration("dictionary"),
                "marker_frame": LaunchConfiguration("marker_frame"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
