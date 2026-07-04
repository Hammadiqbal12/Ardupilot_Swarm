from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_tf_nodes(context):
    """Create tf_odom nodes and static map-to-odom transforms per vehicle."""
    num_vehicles = int(LaunchConfiguration("num_vehicles").perform(context))
    prefix = LaunchConfiguration("vehicle_prefix").perform(context)
    start_index = int(LaunchConfiguration("start_index").perform(context))
    map_frame = LaunchConfiguration("map_frame").perform(context)

    nodes = []
    for offset in range(max(1, num_vehicles)):
        vehicle_index = start_index + offset
        name = f"{prefix}{vehicle_index}"
        ns = name

        tf_node = Node(
            package="misc_nodes",
            executable="tf_odom",
            name=f"tf_odom_{name}",
            namespace=ns,
            remappings=[("odometry", f"/{ns}/odometry")],
            parameters=[{
                "odom_frame": f"{ns}/odom",
                "base_frame": f"{ns}/base_link",
            }],
            output="screen",
        )
        nodes.append(tf_node)

        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_{map_frame}_to_{name}_odom",
            arguments=[
                "0", "0", "0", "0", "0", "0", map_frame, f"{ns}/odom"
            ],
            output="screen",
        )
        nodes.append(static_tf)

    return nodes


def generate_launch_description():
    """Generate map->odom static TFs and tf_odom relays for multiple vehicles."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "num_vehicles",
                default_value="2",
                description="Number of vehicle namespaces to process.",
            ),
            DeclareLaunchArgument(
                "vehicle_prefix",
                default_value="iris",
                description="Vehicle namespace prefix (e.g. iris -> iris1, iris2...).",
            ),
            DeclareLaunchArgument(
                "start_index",
                default_value="1",
                description="Numeric suffix used by the first vehicle.",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Global frame feeding the static transform publishers.",
            ),
            OpaqueFunction(function=_spawn_tf_nodes),
        ]
    )
