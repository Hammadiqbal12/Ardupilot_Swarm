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
```

## 3. Build the Workspace

Build the Gazebo bringup stack and the local helper package:

```bash
colcon build --packages-up-to ardupilot_gz_bringup misc_nodes
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

## 7. Quick Checks

Check that the package can be found:

```bash
ros2 pkg prefix ardupilot_gz_bringup
```

Check available launch arguments:

```bash
ros2 launch ardupilot_gz_bringup iris_forest.launch.py --show-args
```

Run the bringup tests:

```bash
colcon test --packages-select ardupilot_gz_bringup
colcon test-result --verbose
```

## 8. Clean Runtime Processes

If a launch is interrupted and ports remain busy, close old Gazebo, SITL,
MAVProxy, and DDS agent processes before launching again.

```bash
pkill -f "gz sim|ign gazebo|arducopter|mavproxy.py|micro_ros_agent" || true
```
