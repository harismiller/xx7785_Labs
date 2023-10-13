#!/usr/bin/python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team11_chase_object',
            executable='detect_object',
            output = "screen"
        ),
        Node(
            package='team11_chase_object',
            executable='get_object_range',
            output = "screen"
        ),
        Node(
            package='team11_chase_object',
            executable='chase_object',
            output = "screen"
        )
    ])