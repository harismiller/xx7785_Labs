import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,Pose2D

import numpy as np
import math

class ObjectFinder(Node):

    def __init__(self):
        
        self.obj_angle = 0.0

        super().__init__('object_finder')

        self._point_subscriber = self.create_subscription(
                Point,
                '/objCenter',
                self._point_callback,
                10)
        self._scan_subcriber = self.create_subscription(
                LaserScan,
                '/scan',
                self._finder_callback,
                10)
        self._vel_publish = self.create_publisher(
                Pose2D,
                '/objLocation',
                5)
    
    def _point_callback(self, point):
        self.obj_angle = math.atan2((point.x)/250)

    def _finder_callback(self, scan):
        msg = Pose2D()
        msg.theta = self.obj_angle
        print(msg)
        self._vel_publish.publish(msg)

def main():
    rclpy.init()
    object_finder = ObjectFinder()

    while rclpy.ok():
        rclpy.spin_once(object_finder)
    
    object_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
