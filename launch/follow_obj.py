#!/usr/bin/python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team11_object_follower',
            executable='find_object.py',
            output = "screen"
        ),
        Node(
            package='team11_object_follower',
            executable='rotate_robot',
        )
    ])