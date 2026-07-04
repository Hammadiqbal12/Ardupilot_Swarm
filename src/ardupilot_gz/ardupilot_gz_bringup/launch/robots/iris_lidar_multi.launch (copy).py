# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright:
#   2024 ArduPilot.org (base pattern)
#   2025 Your project (multi-vehicle variant)
#
# Launch two iris_with_lidar robots in Gazebo + ROS 2, each with its own
# ArduPilot SITL instance and MAVLink system ID provided by separate files:
#
#   ardupilot_gz_bringup/config/sysid_1.parm   -> SYSID_THISMAV 1
#   ardupilot_gz_bringup/config/sysid_2.parm   -> SYSID_THISMAV 2
#
# Usage (example):
#   ros2 launch ardupilot_gz_bringup multi_iris_sitl.launch.py
#       lidar_dim:=3 lidar_dim2:=3
#       use_gz_tf:=true use_gz_tf2:=true
#
# Notes:
# - Unique ports:
#     Vehicle 1 -> master tcp:127.0.0.1:5760, sitl 127.0.0.1:5501, GCS port 2019
#     Vehicle 2 -> master tcp:127.0.0.1:5762, sitl 127.0.0.1:5502, GCS port 2020
# - Namespaces: /iris and /iris2, with TF frame_prefix "iris/" and "iris2/"
# - The sysid_*.parm files come LAST in 'defaults' so they override properly.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ---------- helpers ----------

def _lc(name, suffix=""):
    """Convenience for LaunchConfiguration('name' + suffix)"""
    return LaunchConfiguration(name + suffix)


def _load_robot_desc(pkg_ardupilot_gz_description, lidar_dim: str):
    """Load iris_with_lidar SDF and swap the lidar model per lidar_dim."""
    sdf_file = os.path.join(
        pkg_ardupilot_gz_description, "models", "iris_with_lidar", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Default to 3D if unknown
    if lidar_dim == "2":
        #robot_desc = robot_desc.replace("<uri>model://lidar_2d</uri>",
        #                                "<uri>model://lidar_2d</uri>")
        bridge_cfg = "iris_2Dlidar_bridge.yaml"
        msg = "Using iris_with_2d_lidar_model"
    elif lidar_dim == "3":
        #robot_desc = robot_desc.replace("<uri>model://lidar_2d</uri>",
        #                                "<uri>model://lidar_3d</uri>")
        bridge_cfg = "iris_3Dlidar_bridge.yaml"
        msg = "Using iris_with_3d_lidar_model"
    else:
        robot_desc = robot_desc.replace("<uri>model://lidar_2d</uri>",
                                        "<uri>model://lidar_3d</uri>")
        bridge_cfg = "iris_3Dlidar_bridge.yaml"
        msg = "ERROR: unknown lidar dimensions! Defaulting to 3d lidar"
    

    return robot_desc, bridge_cfg, msg


# ---------- per-vehicle builders (reused for v1 and v2) ----------

import os, re, tempfile
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def _launch_state_pub_with_bridge(context, suffix: str):
    """
    Build robot_state_publisher + ros_gz_bridge + optional tf relay
    for a given vehicle suffix: "" (vehicle 1) or "2" (vehicle 2).
    """
    lidar_dim = _lc("lidar_dim", suffix).perform(context)
    name = _lc("name", suffix).perform(context)
    use_gz_tf = _lc("use_gz_tf", suffix).perform(context)

    pkg_ardupilot_gz_description = get_package_share_directory("ardupilot_gz_description")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    robot_desc, bridge_cfg, msg = _load_robot_desc(pkg_ardupilot_gz_description, lidar_dim)

    # Default bridge config path
    bridge_cfg_path = os.path.join(pkg_project_bringup, "config", bridge_cfg)

    # Vehicle-2 specific overrides
    if suffix == "2":
        # 1️⃣ Adjust unique FDM port numbers
        robot_desc = robot_desc.replace(
            "<fdm_port_in>9002</fdm_port_in>", "<fdm_port_in>9012</fdm_port_in>"
        )
        
        robot_desc = robot_desc.replace(
            "<odom_frame>iris/odom</odom_frame>", "<odom_frame>iris2/odom</odom_frame>"
        )
        robot_desc = robot_desc.replace(
            "<robot_base_frame>iris/base_link</robot_base_frame>", "<robot_base_frame>iris2/base_link</robot_base_frame>"
        )
        
        

        # 2️⃣ Use a different LiDAR model instance (if available)
        robot_desc = robot_desc.replace(
            "<uri>model://lidar_3d</uri>", "<uri>model://lidar2_3d</uri>"
        )

        # 3️⃣ Duplicate bridge YAML and replace "iris" → "iris2"
        with open(bridge_cfg_path, "r") as f:
            ytxt = f.read()

        # Replace only standalone "iris" tokens
        ytxt = re.sub(r"\biris\b", name, ytxt)

        # Write to temp file and use that instead
        tmp_yaml = tempfile.NamedTemporaryFile(delete=False, suffix=f"_{name}.yaml")
        with open(tmp_yaml.name, "w") as outf:
            outf.write(ytxt)
        bridge_cfg_path = tmp_yaml.name  # override config path

    # robot_state_publisher inside namespace with frame_prefix to avoid TF collisions
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=name,            # /iris or /iris2
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": f"{name}/"},
        ],
    )

    # ros_gz_bridge for topics listed in config yaml
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=name,
        parameters=[{
            "config_file": bridge_cfg_path,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        output="screen",
    )

    # Relay Gazebo TF to namespaced /tf if requested
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        namespace=name,
        arguments=["/gz/tf", "/tf"],
        output="screen",
        respawn=False,
        condition=IfCondition(use_gz_tf),
    )

    event = RegisterEventHandler(
        OnProcessStart(target_action=bridge, on_start=[topic_tools_tf])
    )

    return [
        LogInfo(msg=f"[{name}] {msg}"),
        robot_state_publisher,
        bridge,
        event,
    ]



def _launch_spawn_robot(context, suffix: str):
    """
    Spawn the vehicle in Gazebo using ros_gz_sim create,
    consuming /<name>/robot_description.
    """
    name = _lc("name", suffix).perform(context)
    pos_x = _lc("x", suffix).perform(context)
    pos_y = _lc("y", suffix).perform(context)
    pos_z = _lc("z", suffix).perform(context)
    rot_r = _lc("R", suffix).perform(context)
    rot_p = _lc("P", suffix).perform(context)
    rot_y = _lc("Y", suffix).perform(context)

    # ros_gz_sim create expects an absolute topic
    robot_desc_topic = f"/{name}/robot_description"

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=name,
        arguments=[
            "-world", "",              # current world
            "-param", "",              # default
            "-name", name,
            "-topic", robot_desc_topic,
            "-x", pos_x, "-y", pos_y, "-z", pos_z,
            "-R", rot_r, "-P", rot_p, "-Y", rot_y,
        ],
        output="screen",
    )
    return [spawn_robot]


# ---------- OpaqueFunction wrappers ----------

def launch_state_pub_with_bridge_v1(context):
    return _launch_state_pub_with_bridge(context, "")

def launch_state_pub_with_bridge_v2(context):
    return _launch_state_pub_with_bridge(context, "2")

def launch_spawn_robot_v1(context):
    return _launch_spawn_robot(context, "")

def launch_spawn_robot_v2(context):
    return _launch_spawn_robot(context, "2")


# ---------- launch arguments ----------

def generate_launch_arguments():
    """Define args for both vehicles; v2 args are suffixed with '2'."""
    args = [
        # Vehicle 1
        DeclareLaunchArgument("use_gz_tf", default_value="true",
                              description="Use Gazebo TF relay for vehicle 1."),
        DeclareLaunchArgument("lidar_dim", default_value="3",
                              description="2 or 3 for vehicle 1 lidar."),
        DeclareLaunchArgument("model", default_value="iris_with_lidar",
                              description="Model name/file for vehicle 1."),
        DeclareLaunchArgument("name", default_value="iris",
                              description="Vehicle 1 namespace / instance name."),
        DeclareLaunchArgument("x", default_value="0", description="Vehicle 1 x (m)."),
        DeclareLaunchArgument("y", default_value="0", description="Vehicle 1 y (m)."),
        DeclareLaunchArgument("z", default_value="0.194923", description="Vehicle 1 z (m)."),
        DeclareLaunchArgument("R", default_value="0", description="Vehicle 1 roll (rad)."),
        DeclareLaunchArgument("P", default_value="0", description="Vehicle 1 pitch (rad)."),
        DeclareLaunchArgument("Y", default_value="0", description="Vehicle 1 yaw (rad)."),

        # Vehicle 2
        DeclareLaunchArgument("use_gz_tf2", default_value="true",
                              description="Use Gazebo TF relay for vehicle 2."),
        DeclareLaunchArgument("lidar_dim2", default_value="3",
                              description="2 or 3 for vehicle 2 lidar."),
        DeclareLaunchArgument("model2", default_value="iris_with_lidar",
                              description="Model name/file for vehicle 2."),
        DeclareLaunchArgument("name2", default_value="iris2",
                              description="Vehicle 2 namespace / instance name."),
        DeclareLaunchArgument("x2", default_value="2.0", description="Vehicle 2 x (m)."),
        DeclareLaunchArgument("y2", default_value="0", description="Vehicle 2 y (m)."),
        DeclareLaunchArgument("z2", default_value="0.194923", description="Vehicle 2 z (m)."),
        DeclareLaunchArgument("R2", default_value="0", description="Vehicle 2 roll (rad)."),
        DeclareLaunchArgument("P2", default_value="0", description="Vehicle 2 pitch (rad)."),
        DeclareLaunchArgument("Y2", default_value="0", description="Vehicle 2 yaw (rad)."),
    ]
    return args


# ---------- main launch description ----------

def generate_launch_description():
    """Generate a launch description for two iris quadrotors + two SITL."""
    launch_arguments = generate_launch_arguments()

    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # Common defaults: base vehicle + DDS/UDP
    defaults_common = [
        os.path.join(pkg_ardupilot_sitl, "config", "default_params", "gazebo-iris.parm"),
        os.path.join(pkg_ardupilot_sitl, "config", "default_params", "dds_udp.parm"),
    ]

    # SITL for vehicle 1
    sitl_dds_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ardupilot_sitl"),
                                  "launch", "sitl_dds_udp.launch.py"])
        ]),
        launch_arguments={
            "transport": "udp4",
            "port": "2019",                # GCS UDP port (example)
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "sysid": "1",
             "defaults": os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "gazebo-iris.parm",
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds_udp.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    # SITL for vehicle 2 (unique ports + different sysid override)
    sitl_dds_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ardupilot_sitl"),
                                  "launch", "sitl_dds_udp.launch.py"])
        ]),
        launch_arguments={
            "transport": "udp4",
            "port": "2030",
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "1",
            "sysid": "2",
            "defaults": os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "gazebo-iris.parm",
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds_udp1.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5770",
            "sitl": "127.0.0.1:5511",
        }.items(),
    )

    # Ensure SDF_PATH includes GZ_SIM_RESOURCE_PATH (sdformat_urdf uses SDF_PATH)
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            os.environ["SDF_PATH"] = os.environ["SDF_PATH"] + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Build actions
    ld = LaunchDescription(launch_arguments)

    # SITL instances
    ld.add_action(sitl_dds_1)
    ld.add_action(sitl_dds_2)

    # Vehicle 1 graph (desc/bridge + spawn)
    ld.add_action(OpaqueFunction(function=launch_state_pub_with_bridge_v1))
    ld.add_action(OpaqueFunction(function=launch_spawn_robot_v1))

    # Vehicle 2 graph (desc/bridge + spawn)
    ld.add_action(OpaqueFunction(function=launch_state_pub_with_bridge_v2))
    ld.add_action(OpaqueFunction(function=launch_spawn_robot_v2))

    return ld

