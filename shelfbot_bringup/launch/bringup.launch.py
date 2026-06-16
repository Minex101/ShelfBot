from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Launch the shelfbot_localization package
    localizer = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('shelfbot_navigation'),
            'launch',
            'localizer.launch.py'
        ])
    )

    # Launch the warehouse map server for map topic
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': PathJoinSubstitution([
            FindPackageShare('shelfbot_navigation'),
            'maps',
            'map.yaml'
        ])}]
    )

    # Launch the lifecycle manager to auto-activate the map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }],
        output='screen'
    )

    return LaunchDescription([
        localizer,
        map_server,
        lifecycle_manager,
    ])