import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,Point

import numpy as np
import math

class ObjectFinder(Node):

    def __init__(self):
        super().__init__('object_finder')
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
        
        self._location_publish = self.create_publisher(
                Pose2D,
                '/objLocation',
                5)
        
    def _finder_callback(self, scan):
        msg = Pose2D()
        msg.x = scan
        self._location_publish.publish(msg)

def main():
    rclpy.init()
    object_finder = ObjectFinder()

    while rclpy.ok():
        rclpy.spin_once(object_finder)
    
    object_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
