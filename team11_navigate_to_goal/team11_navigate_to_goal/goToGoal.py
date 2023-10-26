import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,Twist
from nav_msgs.msg import Odometry

import numpy as np

class MoveRobot(Node):

    def __init__(self):

        self.odom = Odometry()

        super().__init__('move_robot')

        self._odom_subcriber = self.create_subscription(
                Odometry,
                '/odom',
                self._odom_callback,
                5)
        
        self._obj_subcriber = self.create_subscription(
                Pose2D,
                '/objLocation',
                self._move_callback,
                5)

        self._move_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                5)
        
    def _move_callback(self, pos):
        msg = Twist()
        msg.x = pos
        self._move_publish.publish(msg)

    def _odom_callback(self, odom):
        self.odom = odom


def main():
    rclpy.init()
    move_robot = MoveRobot()

    while rclpy.ok():
        rclpy.spin_once(move_robot)
    
    move_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
