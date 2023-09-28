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
                'objTracker',
                self._rotation_callback,
                10)
        self._vel_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                5)
    
    def _rotation_callback(self, Point):
        msg = Twist()
        if Point.x > 0:
            msg.angular = Vector3(5,0,0)
        elif Point.x < 0:
            msg.angular = Vector3(-5,0,0)
        msg.angular
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
