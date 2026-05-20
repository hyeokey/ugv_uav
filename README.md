PX4 Gazebo 시뮬레이션에서 카메라가 달린 드론, Ackermann rover, ArUco detector,
정밀착륙 노드, LiDAR 기반 로컬 플래너를 실행하는 명령어 모음.

## px4_msgs 설치

정밀착륙 노드와 로컬 플래너처럼 ROS 2에서 PX4 메시지를 사용하는 노드는 `px4_msgs`가 필요
팀원은 이 워크스페이스의 `src`에 `px4_msgs`를 받은 뒤 같이 빌드하면 됨.

```bash
cd ~/ugv_uav_ws/src
git clone https://github.com/PX4/px4_msgs.git

cd ~/ugv_uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

이미 `src/px4_msgs`가 있으면 clone은 생략하고 빌드만 다시 실행

```bash
cd ~/ugv_uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

설치 확인:

```bash
ros2 pkg prefix px4_msgs
```

정상 예시:

```text
/home/dong/ugv_uav_ws/install/px4_msgs
```

## 공통 ROS 환경 설정

ROS 2 노드를 실행하는 터미널마다 필요한 workspace를 source

```bash
source /opt/ros/humble/setup.bash
source ~/drone_space/ros2_ws/install/setup.bash
source ~/ugv_uav_ws/install/setup.bash
source install/setup.bash
```

## PX4 SITL 실행

### 드론 + Baylands 공원 맵

```bash
cd ~/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=baylands PX4_SYS_AUTOSTART=4014 PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 0
```

### 드론 + Lawn 잔디 맵

```bash
cd ~/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=lawn PX4_SYS_AUTOSTART=4014 PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 0
```

### Ackermann rover 22001 + Lawn 잔디 맵

```bash
cd ~/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=lawn PX4_SYS_AUTOSTART=22001 PX4_GZ_MODEL_POSE="2,0,0.2,0,0,0" ./build/px4_sitl_default/bin/px4 -i 1
```

Rover 콘솔에서 속도/가속도 파라미터를 설정

```bash
param set RO_SPEED_LIM 4
param set RO_ACCEL_LIM 2.5
```

## Sensor Bridge 실행

카메라와 LiDAR 토픽을 ROS 2로 다시 publish

```bash
ros2 launch gz_sensor_bridge sensor_bridges.launch.py \
  world:=lawn \
  drone_model:=x500_mono_cam_down_0 \
  rover_model:=explorer_ackermann_rover_1
```

## ArUco Detector

ArUco marker detector 노드를 실행

```bash
ros2 launch ugv_description aruco_detector.launch.py use_sim_time:=true
```

## 정밀착륙 노드 실행

```bash
source /home/dong/drone_space/ros2_ws/install/setup.bash
source ~/ugv_uav_ws/install/setup.bash

ros2 launch ugv_description aruco_precision_landing.launch.py \
  use_sim_time:=true \
  auto_offboard:=true \
  force_disarm_when_close:=true
```

## LiDAR 로컬 플래너 실행

QGC에서 waypoint를 찍으면 PX4 mission item을 global goal로 받고, LiDAR 기반 local target을 계속 갱신하면서 rover setpoint를 publish

```bash
source /home/dong/drone_space/ros2_ws/install/setup.bash
source ~/ugv_uav_ws/install/setup.bash

ros2 launch ugv_description lidar_local_planner.launch.py \
  use_sim_time:=true \
  px4_ns:=/px4_1 \
  goal_source:=mission \
  heading_source:=course \
  lidar_yaw_offset:=0.0 \
  obstacle_range:=1.5 \
  local_target_distance:=1.5 \
  desired_speed:=0.5 \
  max_speed:=0.6 \
  auto_offboard:=true \
  auto_arm:=true
```

## RViz Marker Topic

```bash
/ugv/local_planner/global_goal_marker
/ugv/local_planner/local_target_marker
/ugv/local_planner/markers
```

빨간 마커는 QGC/global 최종 목표점, 초록 마커는 로컬 플래너가 현재 추종하는 local target

근데 이 부분이 문제가 있음 안나옴. 
