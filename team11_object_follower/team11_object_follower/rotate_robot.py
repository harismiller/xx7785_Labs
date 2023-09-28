#!/usr/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point,Twist,Vector3

import sys

import numpy as np

class RobotRotation(Node):

    def __init__(self):
        super().__init__('robot_rotation')

        self._point_subscriber = self.create_subscription(
                Point,
                'objCenter',
                self._rotation_callback,
                10)
        self._vel_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                5)
    
    def _rotation_callback(self, point):
        msg = Twist()
        if point.x > 20:
            msg.angular.z = -0.3
        elif point.x < -20:
            msg.angular.z = 0.3
        else:
            msg.angular.x = 0.0
        print(msg)
        self._vel_publish.publish(msg)

def main():
    rclpy.init()
    robot_rotation = RobotRotation()

    while rclpy.ok():
        rclpy.spin_once(robot_rotation)
    
    robot_rotation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
