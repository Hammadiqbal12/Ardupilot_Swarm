# file: octomap_builder/launch/multi_octomap_with_merger.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_octomap_nodes(context, *args, **kwargs):
    """
    Dynamically create one octomap_server_node per iris vehicle.

    Topics:
      /iris1/octomap_full, /iris2/octomap_full, ...
    LiDAR topics:
      /iris1/cloud_in, /iris2/cloud_in, ...
    """
    vehicle_count = int(LaunchConfiguration('vehicle_count').perform(context))

    nodes = []

    for i in range(1, vehicle_count + 1):
        ns = f"/iris{i}"
        cloud_topic = f"/iris{i}/cloud_in"

        node = Node(
            package='octomap_server',
            executable='octomap_server_node',
            namespace=ns,              # equivalent to -r __ns:=/irisX
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'resolution': 0.25,
                'max_range': 10.0,
                'publish_free_space': True,
                'filter_speckles': True,
            }],
            # Explicit remap to be very clear:
            remappings=[
                ('cloud_in', cloud_topic),  # equivalent to -r cloud_in:=/irisX/cloud_in
            ],
        )

        nodes.append(node)

    return nodes


def generate_launch_description():
    # How many iris vehicles?
    vehicle_count_arg = DeclareLaunchArgument(
        'vehicle_count',
        default_value='3',
        description='Number of iris vehicles (iris1, iris2, ..., irisN)'
    )

    # OpaqueFunction lets us read vehicle_count and create nodes in Python
    octomap_nodes_action = OpaqueFunction(function=generate_octomap_nodes)

    # Merger node: subscribes to /irisX/octomap_full and publishes /global_octomap_full
    merger_node = Node(
        package='octomap_builder',
        executable='octomap_merger_node',
        name='octomap_merger',
        output='screen',
        parameters=[{
            'vehicle_count': LaunchConfiguration('vehicle_count'),
            'vehicle_prefix': '/iris',
            'octomap_topic_suffix': '/octomap_full',
            'resolution': 0.25,
            'frame_id': 'map',
            'publish_rate': 2.0,
        }],
    )

    return LaunchDescription([
        vehicle_count_arg,
        octomap_nodes_action,
        merger_node,
    ])
