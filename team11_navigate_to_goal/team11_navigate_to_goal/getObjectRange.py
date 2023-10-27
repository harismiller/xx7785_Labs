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
        self._pose_subscriber = self.create_subscription(
            Pose2D,
            '/odomUpdate',
            self.pose_callback,
            10
        )
        self._scan_subcriber = self.create_subscription(
                LaserScan,
                '/scan',
                self._finder_callback,
                scan_qos_profile)
        
        self._location_publish = self.create_publisher(
                Point,
                '/obsLocation',
                10)
        
    def pose_callback(self,pose):
        self.robot_pose = pose

    def _finder_callback(self, scan):
        msg = Point()
        # print(scan.ranges[350])
        scan = np.array(scan.ranges)
        # print(np.shape(scan))
        # print(scan[350])
        min_dist = float(np.nanmin(scan))
        # print(min_dist)
        min_angle_local = np.nanargmin(scan)*1.5
        # Convert the robot's orientation to radians
        robot_orientation_rad = self.robot_pose.theta

        # Convert the local angle to radians
        angle_rad = np.deg2rad(min_angle_local)

        # Calculate the global angle by adding the robot's orientation
        global_angle_rad = robot_orientation_rad + angle_rad
        # print(min_dist,min_angle_local,angle_rad,global_angle_rad)
        # Calculate the global X and Y coordinates of the point
        msg.x = self.robot_pose.x + min_dist * np.cos(global_angle_rad)
        msg.y = self.robot_pose.y + min_dist * np.sin(global_angle_rad)
        # print(msg.x,msg.y,min_dist)
        msg.z = min_dist
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
