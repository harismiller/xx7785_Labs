import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,Pose2D
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import math
import time

class ObjectFinder(Node):

    def __init__(self):
        
        self.obj_angle = 0.0
        self.scan = LaserScan()
        # scan.ranges = []
        # self.scan = scan
        self.DEFAULT_RADIAL_DIST = 0.0

        super().__init__('object_finder')
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
                self._finder_callback,
                scan_qos_profile)
        # time.sleep(2)
        self._point_subscriber = self.create_subscription(
                Point,
                '/objCenter',
                self._point_callback,
                10)
        self._location_publish = self.create_publisher(
                Pose2D,
                '/objLocation',
                5)
    
    def _point_callback(self, point):
        self.obj_angle = math.atan2(point.x,250)

        msg = Pose2D()
        index = int((self.obj_angle+(2*math.pi))%(2*math.pi))
        radial_dist = self.DEFAULT_RADIAL_DIST
        if not math.isnan(self.scan.ranges[index]):
            radial_dist = self.scan.ranges[index]
        msg.x = radial_dist
        msg.theta = self.obj_angle
        print(msg)
        self._location_publish.publish(msg)

    def _finder_callback(self, scan):
        self.scan = scan
        

def main():
    rclpy.init()
    object_finder = ObjectFinder()

    while rclpy.ok():
        rclpy.spin_once(object_finder)
    
    object_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
