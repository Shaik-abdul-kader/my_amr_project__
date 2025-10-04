from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('my_robo_project').find('my_robo_project')
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        # --- Declare args ---
        DeclareLaunchArgument(
            name='map',
            default_value=os.path.join(pkg_share, 'maps', 'myroom_map.yaml'),
            description='Path to map yaml file'
        ),

        DeclareLaunchArgument(
            name='params_file',
            default_value=os.path.join(pkg_share, 'config', 'costmap_room.yaml'),  #costmap_room.yaml
            description='Path to Nav2 parameters file including costmaps'
        ),

        # --- Map server node ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}]
        ),

        # --- AMCL node ---
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'amcl.yaml'),
                {'use_sim_time': True}
            ]
        ),

        # lifecycle manager to bring nodes active
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ])

