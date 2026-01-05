import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    config_path = os.path.join(os.path.dirname(__file__), 'costmap.yaml')
    map_path = os.path.join(os.path.dirname(__file__), 'warehouse.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom',
            arguments=["--frame-id", "map", "--child-frame-id", "odom", "--x", "-7.6", "--y", "-8.5", "--yaw", "0.7"],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_baselink',
            arguments=["--frame-id", "odom", "--child-frame-id", "base_link"],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_cartercamerastereoright',
            arguments=["--frame-id", "base_link", "--child-frame-id", "carter_camera_stereo_right", "--z", "0.25", "--qx","-0.501", "--qy", "0.501", "--qz", "-0.499", "--qw", "0.499"],
            output='screen'
        ),
        # Costmap node under the "costmap" namespace so its fully-qualified name becomes /costmap/costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            parameters=[config_path],
            output='screen'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[
                config_path, 
                {"yaml_filename": map_path}
                ],
            output='screen'
        ),

        # Lifecycle manager auto-configures and activates the costmap node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            parameters=[
                {
                    'autostart': True,
                    'node_names': ['map_server', 'costmap/costmap',]
                }
            ]
        ),
    ])
