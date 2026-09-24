"""Starts rviz2 with a configuration that uses the Cinematographer view controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('rviz2_cinematographer_view_controller')
    default_config = os.path.join(share_dir, 'launch', 'demo.rviz')

    load_config_arg = DeclareLaunchArgument(
        'load_config', default_value='true',
        description='Load the demo rviz configuration.')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_config,
        description='Path to the rviz configuration file.')
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Start rviz2 in gdb.')

    launch_prefix = PythonExpression(
        ["'gdb --ex run --args' if '", LaunchConfiguration('debug'), "' == 'true' else ''"])

    rviz_with_config = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        prefix=launch_prefix,
        condition=IfCondition(LaunchConfiguration('load_config')),
    )

    rviz_without_config = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        prefix=launch_prefix,
        condition=UnlessCondition(LaunchConfiguration('load_config')),
    )

    return LaunchDescription([
        load_config_arg,
        rviz_config_arg,
        debug_arg,
        rviz_with_config,
        rviz_without_config,
    ])
