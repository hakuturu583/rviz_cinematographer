"""Starts rviz2 and the Rviz Cinematographer rqt plugin."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('rviz_cinematographer_gui')
    default_rviz_config = os.path.join(share_dir, 'launch', 'rviz_cinematographer_gui.rviz')
    default_trajectory = os.path.join(share_dir, 'trajectories', 'example_trajectory.yaml')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='Path to the rviz configuration file.')
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file', default_value=default_trajectory,
        description='Trajectory (yaml) that is loaded on start up.')
    start_recorder_arg = DeclareLaunchArgument(
        'start_recorder', default_value='true',
        description='Start the video recorder together with the GUI.')

    # Start RViz separately, because the rqt rviz plugin can't handle published trajectories.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    gui = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rviz_cinematographer_gui',
        output='screen',
        arguments=[
            '-s', 'rviz_cinematographer_gui/RvizCinematographerGUI',
            '--args',
            '--trajectory-file', LaunchConfiguration('trajectory_file'),
            '--start-recorder', LaunchConfiguration('start_recorder'),
        ],
    )

    return LaunchDescription([
        rviz_config_arg,
        trajectory_file_arg,
        start_recorder_arg,
        rviz,
        gui,
    ])
