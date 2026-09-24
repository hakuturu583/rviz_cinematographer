"""Starts the video recorder node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2_cinematographer_video_recorder',
            executable='video_recorder_node',
            name='video_recorder',
            output='screen',
        ),
    ])
