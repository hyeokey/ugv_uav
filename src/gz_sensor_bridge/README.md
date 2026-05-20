# gz_sensor_bridge

Direct Gazebo Transport to ROS 2 sensor bridges for the UAV/UGV docking simulation.

## Nodes

- `gz_camera_bridge_node`
  - Subscribes: Gazebo `gz.msgs.Image`, `gz.msgs.CameraInfo`
  - Publishes: ROS 2 `sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo`
  - Defaults:
    - `/drone/down_camera/image_raw`
    - `/drone/down_camera/camera_info`

- `gz_lidar_bridge_node`
  - Subscribes: Gazebo `gz.msgs.LaserScan`
  - Publishes: ROS 2 `sensor_msgs/msg/LaserScan`
  - Default:
    - `/ugv/lidar/scan`

## Build

```bash
cd ~/ugv_uav_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch gz_sensor_bridge sensor_bridges.launch.py \
  world:=baylands \
  drone_model:=x500_mono_cam_down_0 \
  rover_model:=explorer_rover_1
```

Check the exact Gazebo model instance names with:

```bash
gz topic -l
```

If PX4 spawned a different instance name, pass it through `drone_model:=...` or `rover_model:=...`.
