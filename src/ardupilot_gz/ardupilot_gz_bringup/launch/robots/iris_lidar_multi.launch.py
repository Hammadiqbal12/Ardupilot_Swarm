# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright:
#   2024 ArduPilot.org (base pattern)
#   2025 Your project (multi-vehicle variant)
#
# Launch multiple iris_with_lidar robots in Gazebo + ROS 2, each with its own
# ArduPilot SITL instance and auto-generated DDS UDP defaults/ports.

import os
import re
import tempfile

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


def _lc(name):
    """Convenience for LaunchConfiguration('name')."""
    return LaunchConfiguration(name)


def _resolve(context, name, cast=str):
    """Resolve a LaunchConfiguration value and cast it."""
    return cast(_lc(name).perform(context))


def _load_robot_desc(pkg_ardupilot_gz_description, lidar_dim: str):
    """Load iris_with_lidar SDF and swap the lidar model per lidar_dim."""
    sdf_file = os.path.join(
        pkg_ardupilot_gz_description, "models", "iris_with_lidar", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    if lidar_dim == "2":
        bridge_cfg = "iris_2Dlidar_bridge.yaml"
        msg = "Using iris_with_2d_lidar_model"
    elif lidar_dim == "3":
        bridge_cfg = "iris_3Dlidar_bridge.yaml"
        msg = "Using iris_with_3d_lidar_model"
    else:
        robot_desc = robot_desc.replace(
            "<uri>model://lidar_2d</uri>", "<uri>model://lidar_3d</uri>"
        )
        bridge_cfg = "iris_3Dlidar_bridge.yaml"
        msg = "ERROR: unknown lidar dimensions! Defaulting to 3d lidar"

    return robot_desc, bridge_cfg, msg


def _customize_lidar_model(pkg_ardupilot_gz_description: str,
                           lidar_dim: str,
                           name: str):
    """Create a lidar model copy with unique TF/topic names per vehicle."""
    model_name = "lidar_2d" if lidar_dim == "2" else "lidar_3d"
    template_path = os.path.join(
        pkg_ardupilot_gz_description, "models", model_name, "model.sdf"
    )
    with open(template_path, "r") as infp:
        lidar_sdf = infp.read()

    # Replace plain "iris" tokens and iris-prefixed frames/topics.
    lidar_sdf = lidar_sdf.replace("iris/", f"{name}/")
    lidar_sdf = re.sub(r"\biris\b", name, lidar_sdf)

    # Ensure base_scan frame/topic names include the namespace.
    lidar_sdf = lidar_sdf.replace(
        "<gz_frame_id>base_scan</gz_frame_id>",
        f"<gz_frame_id>{name}/base_scan</gz_frame_id>",
    )
    lidar_sdf = lidar_sdf.replace(
        "<topic>lidar</topic>", f"<topic>{name}/lidar</topic>"
    )

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=f"_{name}_{model_name}.sdf")
    tmp.close()
    with open(tmp.name, "w") as outfp:
        outfp.write(lidar_sdf)

    return tmp.name, model_name


def _write_bridge_config(template_path: str, name: str) -> str:
    """Create a temporary bridge config with namespace substitutions."""
    with open(template_path, "r") as f:
        cfg_text = f.read()
    cfg_text = re.sub(r"\biris\b", name, cfg_text)
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=f"_{name}_bridge.yaml")
    tmp.close()
    with open(tmp.name, "w") as out:
        out.write(cfg_text)
    return tmp.name


def _prepare_robot_resources(name: str,
                             lidar_dim: str,
                             pkg_ardupilot_gz_description: str,
                             pkg_bringup: str,
                             fdm_port: int):
    """Return the robot description and bridge config path."""
    robot_desc, bridge_cfg, msg = _load_robot_desc(
        pkg_ardupilot_gz_description, lidar_dim
    )
    robot_desc = robot_desc.replace(
        "<odom_frame>iris/odom</odom_frame>",
        f"<odom_frame>{name}/odom</odom_frame>",
    )
    robot_desc = robot_desc.replace(
        "<robot_base_frame>iris/base_link</robot_base_frame>",
        f"<robot_base_frame>{name}/base_link</robot_base_frame>",
    )
    robot_desc = robot_desc.replace(
        "<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{fdm_port}</fdm_port_in>"
    )

    lidar_override_path, lidar_model_name = _customize_lidar_model(
        pkg_ardupilot_gz_description, lidar_dim, name
    )

    if lidar_dim == "2":
        robot_desc = robot_desc.replace(
            "<uri>model://lidar_3d</uri>", "<uri>model://lidar_2d</uri>"
        )

    robot_desc = robot_desc.replace(
        f"<uri>model://{lidar_model_name}</uri>",
        f"<uri>file://{lidar_override_path}</uri>",
    )

    bridge_template = os.path.join(pkg_bringup, "config", bridge_cfg)
    bridge_cfg_path = _write_bridge_config(bridge_template, name)
    return robot_desc, bridge_cfg_path, msg


def _create_dds_defaults(template_path: str, port: int, name: str) -> str:
    """Clone the DDS defaults file with a unique UDP port."""
    with open(template_path, "r") as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        if line.startswith("DDS_UDP_PORT"):
            new_lines.append(f"DDS_UDP_PORT {port}\n")
        else:
            new_lines.append(line)

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=f"_{name}_dds.parm")
    tmp.close()
    with open(tmp.name, "w") as out:
        out.writelines(new_lines)
    return tmp.name


def _build_vehicle_graph(name: str,
                         lidar_dim: str,
                         pose_xyz,
                         rpy,
                         pkg_ardupilot_gz_description: str,
                         pkg_bringup: str,
                         fdm_port: int):
    """Create robot_state_publisher, bridge, TF relay, and spawn actions."""
    robot_desc, bridge_cfg_path, msg = _prepare_robot_resources(
        name, lidar_dim, pkg_ardupilot_gz_description, pkg_bringup, fdm_port
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=name,
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": f"{name}/"},
        ],
    )

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

    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        namespace=name,
        arguments=["/gz/tf", "/tf"],
        output="screen",
        respawn=False,
        condition=IfCondition(_lc("use_gz_tf")),
    )

    event = RegisterEventHandler(
        OnProcessStart(target_action=bridge, on_start=[topic_tools_tf])
    )

    robot_desc_topic = f"/{name}/robot_description"
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=name,
        arguments=[
            "-world", "",
            "-param", "",
            "-name", name,
            "-topic", robot_desc_topic,
            "-x", str(pose_xyz[0]),
            "-y", str(pose_xyz[1]),
            "-z", str(pose_xyz[2]),
            "-R", str(rpy[0]),
            "-P", str(rpy[1]),
            "-Y", str(rpy[2]),
        ],
        output="screen",
    )

    return [
        LogInfo(msg=f"[{name}] {msg}"),
        robot_state_publisher,
        bridge,
        event,
        spawn_robot,
    ]


def _build_sitl_instance(defaults: str,
                         instance_index: int,
                         dds_port: int,
                         master_port: int,
                         sitl_port: int,
                         sysid: int):
    """Return the IncludeLaunchDescription that starts SITL + DDS."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ardupilot_sitl"),
                "launch",
                "sitl_dds_udp.launch.py",
            ])
        ]),
        launch_arguments={
            "transport": "udp4",
            "port": str(dds_port),
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": str(instance_index),
            "sysid": str(sysid),
            "defaults": defaults,
            "sim_address": "127.0.0.1",
            "master": f"tcp:127.0.0.1:{master_port}",
            "sitl": f"127.0.0.1:{sitl_port}",
        }.items(),
    )


def _launch_all_vehicles(context):
    """Create per-vehicle actions based on the requested fleet size."""
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_ardupilot_gz_description = get_package_share_directory("ardupilot_gz_description")

    gazebo_defaults = os.path.join(
        pkg_ardupilot_sitl, "config", "default_params", "gazebo-iris.parm"
    )
    dds_template = os.path.join(
        pkg_ardupilot_sitl, "config", "default_params", "dds_udp.parm"
    )

    num_vehicles = max(1, _resolve(context, "num_vehicles", int))
    prefix = _lc("vehicle_prefix").perform(context)
    start_index = _resolve(context, "start_index", int)
    start_x = _resolve(context, "start_x", float)
    x_spacing = _resolve(context, "x_spacing", float)
    pos_y = _resolve(context, "y", float)
    pos_z = _resolve(context, "z", float)
    roll = _lc("R").perform(context)
    pitch = _lc("P").perform(context)
    yaw = _lc("Y").perform(context)
    lidar_dim = _lc("lidar_dim").perform(context)

    dds_base = _resolve(context, "dds_udp_base_port", int)
    dds_step = _resolve(context, "dds_udp_port_step", int)
    master_base = _resolve(context, "master_base_port", int)
    master_step = _resolve(context, "master_port_step", int)
    sitl_base = _resolve(context, "sitl_base_port", int)
    sitl_step = _resolve(context, "sitl_port_step", int)
    fdm_base = _resolve(context, "fdm_base_port", int)
    fdm_step = _resolve(context, "fdm_port_step", int)

    actions = []
    for offset in range(num_vehicles):
        vehicle_index = start_index + offset
        name = f"{prefix}{vehicle_index}"

        pos_x = start_x + x_spacing * offset
        pose = (pos_x, pos_y, pos_z)
        rpy = (roll, pitch, yaw)

        dds_port = dds_base + dds_step * offset
        master_port = master_base + master_step * offset
        sitl_port = sitl_base + sitl_step * offset
        fdm_port = fdm_base + fdm_step * offset
        sysid = vehicle_index

        dds_defaults = _create_dds_defaults(dds_template, dds_port, name)
        defaults = f"{gazebo_defaults},{dds_defaults}"

        sitl_action = _build_sitl_instance(
            defaults,
            offset,
            dds_port,
            master_port,
            sitl_port,
            sysid,
        )
        actions.append(sitl_action)

        actions.extend(
            _build_vehicle_graph(
                name,
                lidar_dim,
                pose,
                rpy,
                pkg_ardupilot_gz_description,
                pkg_bringup,
                fdm_port,
            )
        )

    return actions


def generate_launch_arguments():
    """Define common launch arguments used by the multi-vehicle bringup."""
    return [
        DeclareLaunchArgument(
            "num_vehicles",
            default_value="2",
            description="Number of Iris vehicles to spawn (iris1, iris2, ...).",
        ),
        DeclareLaunchArgument(
            "vehicle_prefix",
            default_value="iris",
            description="Prefix used for namespaces, TF, and model names.",
        ),
        DeclareLaunchArgument(
            "start_index",
            default_value="1",
            description="Numeric suffix used by the first vehicle (iris<start_index>).",
        ),
        DeclareLaunchArgument(
            "start_x",
            default_value="0.0",
            description="Initial X coordinate in meters for the first vehicle.",
        ),
        DeclareLaunchArgument(
            "x_spacing",
            default_value="1.0",
            description="Spacing in meters applied along +X between vehicles.",
        ),
        DeclareLaunchArgument("y", default_value="0.0", description="Y pose for all vehicles (m)."),
        DeclareLaunchArgument(
            "z",
            default_value="0.194923",
            description="Z pose for all vehicles (m).",
        ),
        DeclareLaunchArgument("R", default_value="0", description="Roll (rad)."),
        DeclareLaunchArgument("P", default_value="0", description="Pitch (rad)."),
        DeclareLaunchArgument("Y", default_value="0", description="Yaw (rad)."),
        DeclareLaunchArgument(
            "lidar_dim",
            default_value="3",
            description="2 or 3 for all iris lidar models.",
        ),
        DeclareLaunchArgument(
            "use_gz_tf",
            default_value="true",
            description="Whether to relay Gazebo TF frames into each namespace.",
        ),
        DeclareLaunchArgument(
            "dds_udp_base_port",
            default_value="2019",
            description="Starting DDS UDP port (updates DDS defaults per vehicle).",
        ),
        DeclareLaunchArgument(
            "dds_udp_port_step",
            default_value="11",
            description="Increment applied to DDS UDP ports per vehicle.",
        ),
        DeclareLaunchArgument(
            "master_base_port",
            default_value="5760",
            description="Base MAVLink master TCP port.",
        ),
        DeclareLaunchArgument(
            "master_port_step",
            default_value="10",
            description="Increment applied to MAVLink master TCP ports.",
        ),
        DeclareLaunchArgument(
            "sitl_base_port",
            default_value="5501",
            description="Base ArduPilot SITL TCP port.",
        ),
        DeclareLaunchArgument(
            "sitl_port_step",
            default_value="10",
            description="Increment applied to SITL TCP ports.",
        ),
        DeclareLaunchArgument(
            "fdm_base_port",
            default_value="9002",
            description="Base Gazebo FDM port used by the ArduPilot plugin.",
        ),
        DeclareLaunchArgument(
            "fdm_port_step",
            default_value="10",
            description="Increment applied to Gazebo FDM ports.",
        ),
    ]


def generate_launch_description():
    """Generate a launch description for an arbitrary number of iris quadrotors."""
    launch_arguments = generate_launch_arguments()

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            os.environ["SDF_PATH"] = os.environ["SDF_PATH"] + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    ld = LaunchDescription(launch_arguments)
    ld.add_action(OpaqueFunction(function=_launch_all_vehicles))
    return ld
