#!/usr/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,Pose2D
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import math
import time

class WallFinder(Node):

    def __init__(self):
        time.sleep(2)
        self.obj_angle = 0.0
        self.scan = LaserScan()
        self.scan.ranges = [0.0]*360
        # self.scan = scan
        self.radial_dist = 5.0
        # self.DEFAULT_RADIAL_DIST = 0.0

        super().__init__('wall_finder')
        #Set up QoS Profiles for passing images over WiFi
        scan_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self._scan_subcriber = self.create_subscription(
                LaserScan,
                '/scan',
                self._point_callback,
                scan_qos_profile)
        # time.sleep(2)
        self._sign_subscriber = self.create_subscription(
                Point,
                '/sign',
                self._sign_callback,
                10)
        self._location_publish = self.create_publisher(
                Pose2D,
                '/wallLocation',
                5)
    
    def _sign_callback(self,sign):
        self.sign_type = int(sign.x)

    def get_min_angle(self,filter,angle):
        comp = np.multiply(filter,np.cos(angle))
        avg = np.mean(comp)
        dist = comp - avg
        return angle[np.argmin(dist)]

    def _point_callback(self, scan):
        self.scan = scan
        msg = Pose2D()
        angle_range = [0,1,2,-2,-1]
        delta_angle = self.scan.angle_increment
        actual_angle = np.array(angle_range)*delta_angle
        # print(np.array(self.scan.ranges)[0:2])
        filter_ranges = np.array(self.scan.ranges)[angle_range]
        # print(filter_ranges)
        min_angle = self.get_min_angle(filter_ranges,actual_angle)
        # min_angle = angle_range[np.argmin(filter_ranges)]
        # min_angle_rad = delta_angle*min_angle
        min_distance = np.min(filter_ranges)
        # print(type(min_distance))
        msg.x = float(min_distance)
        msg.theta = min_angle
        print(msg)
        self._location_publish.publish(msg)

    def _finder_callback(self, scan):
        self.scan = scan
        

def main():
    rclpy.init()
    wall_finder = WallFinder()

    while rclpy.ok():
        rclpy.spin_once(wall_finder)
    
    wall_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
