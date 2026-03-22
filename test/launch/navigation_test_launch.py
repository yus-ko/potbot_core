"""
E2E ナビゲーションテスト用ランチファイル
Gazebo + Navigation2 を起動してテストを実行する
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions


def generate_launch_description():
    potbot_example_dir = get_package_share_directory('potbot_example')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(potbot_example_dir, 'maps', 'willowgarage.yaml')
    params_file = os.path.join(potbot_example_dir, 'params', 'burger.yaml')

    # Gazebo (headless)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(potbot_example_dir, 'launch', 'gazbeo',
                         'turtlebot3_with_garage.launch.py')
        ),
        launch_arguments={'gui': 'false'}.items()
    )

    # Navigation2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': params_file,
        }.items()
    )

    return (
        LaunchDescription([
            gazebo,
            TimerAction(period=10.0, actions=[nav2]),
            launch_testing.actions.ReadyToTest()
        ]),
        {}
    )
