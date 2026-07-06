# Tutorial: ArduPilot Swarm ROS 2 Gazebo Workspace

This guide assumes you are using Ubuntu 22.04 with ROS 2 Humble, matching the
standard ArduPilot ROS 2 Gazebo workflow.

## 1. Read the Upstream Setup First

Start with the official ArduPilot documentation:

- ROS 2 install: https://ardupilot.org/dev/docs/ros2-install.html
- ROS 2 SITL: https://ardupilot.org/dev/docs/ros2-sitl.html
- ROS 2 Gazebo: https://ardupilot.org/dev/docs/ros2-gazebo.html

Use those pages to install system dependencies, configure ROS 2, install or
build the Micro XRCE-DDS Agent, and understand the normal single-vehicle SITL
flow. The Micro XRCE-DDS Agent source/build folders are intentionally not kept
in this repository.

Do not repeat upstream commands that clone ArduPilot, `ardupilot_gz`,
`ros_gz`, or other repositories into `src/`. This repository already includes
the source tree needed for this customized workspace.

## 2. Prepare the Workspace

Clone this repository and enter the workspace:

```bash
git clone git@github.com:Hammadiqbal12/Ardupilot_Swarm.git ~/ardu_ws
cd ~/ardu_ws
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Install dependencies for the packages already present in `src/`:

```bash
cd src/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
cd ../..
rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt install -y ros-humble-octomap ros-humble-octomap-msgs ros-humble-octomap-server
```

## 3. Build the Workspace

Build the Gazebo bringup stack, the local helper package, and the OctoMap
mapping package:

```bash
colcon build --packages-up-to ardupilot_gz_bringup misc_nodes octomap_builder
```

Source the workspace after the build:

```bash
source install/setup.bash
```

For a clean terminal later, source both ROS 2 and the workspace:

```bash
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 4. Launch the Swarm Simulation

Basic Iris forest swarm launch:

```bash
ros2 launch ardupilot_gz_bringup iris_forest.launch.py num_vehicles:=5
```

This starts multiple Iris vehicles with lidar in the Gazebo forest world. Each
vehicle is assigned separate namespaces, ports, and IDs so multiple SITL
instances can run at the same time.

## 5. Optional NVIDIA / GPU Gazebo Launch

Keep the normal command as the default. If Gazebo GUI has rendering problems or
does not select the NVIDIA GPU, use this prefix:

```bash
QT_QPA_PLATFORM=xcb \
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
__VK_LAYER_NV_optimus=NVIDIA_only \
LIBGL_ALWAYS_SOFTWARE=0 \
GZ_RENDER_ENGINE=ogre2 \
ros2 launch ardupilot_gz_bringup iris_forest.launch.py num_vehicles:=5
```

## 6. Common Launch Arguments

Use these arguments to control the swarm layout:

```bash
ros2 launch ardupilot_gz_bringup iris_forest.launch.py \
  num_vehicles:=5 \
  start_x:=-8.0 \
  x_spacing:=2.0 \
  lidar_dim:=3
```

Argument summary:

- `num_vehicles`: number of drones to launch.
- `start_x`: X position of the first drone.
- `x_spacing`: spacing between drones along the X axis.
- `y`: shared Y position for all drones.
- `z`: shared Z position for all drones.
- `R`, `P`, `Y`: initial roll, pitch, and yaw.
- `lidar_dim`: `3` for 3D lidar or `2` for 2D lidar.
- `use_gz_tf`: relay Gazebo TF into ROS 2 TF when set to `true`.

## 7. Run Multi-Drone OctoMap Mapping

The `octomap_builder` package is used after the swarm is running. It starts one
`octomap_server_node` per drone and subscribes to each lidar point cloud topic:

- `/iris1/cloud_in`
- `/iris2/cloud_in`
- `/iris3/cloud_in`
- and so on up to the configured vehicle count

Each drone gets an individual OctoMap topic under its own namespace, such as
`/iris1/octomap_full`. The merger node also publishes one combined global map:

- `/global_octomap_full`

Launch the swarm first:

```bash
ros2 launch ardupilot_gz_bringup iris_forest.launch.py num_vehicles:=5
```

Then open a second terminal, source the workspace, and start the TF helper
nodes from `misc_nodes`:

```bash
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch misc_nodes tf_tree.launch.py num_vehicles:=5
```

This launch creates the transform tree needed by the mapping pipeline:

- static `map -> irisN/odom` transforms
- dynamic `irisN/odom -> irisN/base_link` transforms from each drone odometry

Then open a third terminal, source the workspace, and start the OctoMap nodes:

```bash
cd ~/ardu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch octomap_builder multi_octomap_with_merger.launch.py vehicle_count:=5
```

Keep both mapping arguments aligned with the swarm size. For example, if the
swarm launch uses `num_vehicles:=8`, run:

```bash
ros2 launch misc_nodes tf_tree.launch.py num_vehicles:=8
ros2 launch octomap_builder multi_octomap_with_merger.launch.py vehicle_count:=8
```

The `misc_nodes` package is helpful with this mapping setup because it can
create the transforms between each Iris odometry frame and the shared `map`
frame. Those transforms are needed so the per-drone lidar data and OctoMaps line
up in the same global frame.

## 8. Demo Video

This video shows the setup running and demonstrates UAV control through
QGroundControl:

[![ArduPilot Swarm ROS 2 Gazebo demo](https://img.youtube.com/vi/mojc7Xz_36E/0.jpg)](https://www.youtube.com/watch?v=mojc7Xz_36E)

## 9. Quick Checks

Check that the package can be found:

```bash
ros2 pkg prefix ardupilot_gz_bringup
ros2 pkg prefix octomap_builder
```

Check available launch arguments:

```bash
ros2 launch ardupilot_gz_bringup iris_forest.launch.py --show-args
```

Run the bringup tests:

```bash
colcon test --packages-select ardupilot_gz_bringup
colcon test --packages-select octomap_builder
colcon test-result --verbose
```

## 10. Clean Runtime Processes

If a launch is interrupted and ports remain busy, close old Gazebo, SITL,
MAVProxy, and DDS agent processes before launching again.

```bash
pkill -f "gz sim|ign gazebo|arducopter|mavproxy.py|micro_ros_agent" || true
```
