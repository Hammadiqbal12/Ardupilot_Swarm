# ArduPilot Swarm ROS 2 Gazebo Workspace

This repository contains a customized ROS 2 / Gazebo workspace based on the ArduPilot ROS 2 Gazebo setup:

https://ardupilot.org/dev/docs/ros2-gazebo.html

The repository focuses on the workspace `src/` tree: ArduPilot, Gazebo, ROS 2 bridge, SITL model, and custom swarm packages needed to run multi-drone simulation experiments. Build, install, log, and runtime output are intentionally left out and can be recreated locally.

## Layout

- `src/ardupilot`: ArduPilot with ROS 2 SITL support and swarm-related parameter additions.
- `src/ardupilot_gz`: ArduPilot Gazebo launch, description, application, and world packages.
- `src/ardupilot_gazebo`: ArduPilot Gazebo plugin package.
- `src/ardupilot_sitl_models`: SITL Gazebo models.
- `src/ros_gz`: ROS 2 Gazebo bridge packages.
- `src/micro_ros_agent`: micro-ROS agent.
- `src/sdformat_urdf`: SDFormat URDF conversion packages.
- `src/misc_nodes`: custom helper nodes.

## Build

```bash
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to ardupilot_gz_bringup
```

## Run

```bash
cd ~/ardu_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

The upstream dependency import file is preserved at `src/ardupilot_gz/ros2_gz.repos`, and nested `.gitmodules` files are kept so dependency origins remain documented.
