from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration("world")
    drone_model = LaunchConfiguration("drone_model")
    rover_model = LaunchConfiguration("rover_model")
    rover_lidar_link = LaunchConfiguration("rover_lidar_link")
    rover_lidar_sensor = LaunchConfiguration("rover_lidar_sensor")

    gz_image_topic = [
        "/world/", world,
        "/model/", drone_model,
        "/link/camera_link/sensor/camera/image",
    ]
    gz_camera_info_topic = [
        "/world/", world,
        "/model/", drone_model,
        "/link/camera_link/sensor/camera/camera_info",
    ]
    gz_lidar_topic = [
        "/world/", world,
        "/model/", rover_model,
        "/link/", rover_lidar_link,
        "/sensor/", rover_lidar_sensor,
        "/scan",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="baylands"),
        DeclareLaunchArgument("drone_model", default_value="x500_mono_cam_down_0"),
        DeclareLaunchArgument("rover_model", default_value="explorer_rover_1"),
        DeclareLaunchArgument("rover_lidar_link", default_value="lidar_sensor_link"),
        DeclareLaunchArgument("rover_lidar_sensor", default_value="lidar"),
        DeclareLaunchArgument("drone_camera_frame", default_value="drone_down_camera"),
        DeclareLaunchArgument("rover_lidar_frame", default_value="ugv_lidar"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="gz_sensor_bridge",
            executable="gz_camera_bridge_node",
            name="gz_camera_bridge",
            output="screen",
            parameters=[{
                "gz_image_topic": gz_image_topic,
                "gz_camera_info_topic": gz_camera_info_topic,
                "ros_image_topic": "/drone/down_camera/image_raw",
                "ros_camera_info_topic": "/drone/down_camera/camera_info",
                "frame_id": LaunchConfiguration("drone_camera_frame"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
        Node(
            package="gz_sensor_bridge",
            executable="gz_lidar_bridge_node",
            name="gz_lidar_bridge",
            output="screen",
            parameters=[{
                "gz_lidar_topic": gz_lidar_topic,
                "ros_lidar_topic": "/ugv/lidar/scan",
                "frame_id": LaunchConfiguration("rover_lidar_frame"),
                "scan_time": 0.05,
                "stamp_from_gz_header": False,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
