import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    map_yaml_path = LaunchConfiguration('map_yaml_path')

    map_yaml_path_arg = DeclareLaunchArgument(
        'map_yaml_path',
        default_value='/workspaces/isaac_ros-dev/src/shelfbot_navigation/maps/map.yaml',
        description='Path to the map YAML file'
    )

    occupancy_grid_localizer_node = ComposableNode(
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        name='occupancy_grid_localizer',
        parameters=[{
            'loc_result_frame': 'map',
            'map_yaml_path': map_yaml_path,
            'image': 'map.png',
            'resolution': 0.05,
            'origin': [-11.975, -17.975, 0.0],
            'occupied_thresh': 0.65,
            'min_scan_fov_degrees': 100.0,
        }]
    )

    laserscan_to_flatscan_node = ComposableNode(
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode',
        name='laserscan_to_flatscan',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('scan', '/scan'),
            ('flatscan', 'flatscan_localization'),
        ]
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='forklift_localizer_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            occupancy_grid_localizer_node,
            laserscan_to_flatscan_node,
        ],
        output='screen'
    )

    return LaunchDescription([
        map_yaml_path_arg,
        container,
    ])