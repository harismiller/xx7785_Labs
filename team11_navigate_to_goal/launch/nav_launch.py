#!/usr/bin/python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team11_navigate_to_goal',
            executable='printFixedOdometry',
            output = "screen"
        ),
        Node(
            package='team11_navigate_to_goal',
            executable='getObjectRange',
            output = "screen"
        ),
        Node(
            package='team11_navigate_to_goal',
            executable='goToGoal',
            output = "screen"
        )
    ])