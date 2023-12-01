#!/usr/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,Twist,Point

import numpy as np

class MoveRobot(Node):

    def __init__(self):

        # Variables
        self.e_lin_prev = 0

        super().__init__('move_robot')

        self._pose_subscriber = self.create_subscription(
                Pose2D,
                '/wallLocation',
                self._chase_callback,
                10)
        self._vel_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                5)

    def _chase_callback(self, pose):
        msg = Twist()
        r = pose.x
        # print(r,'goal_dist')
        theta = pose.theta

        # Angular Controller
        Kp_ang = 1.5
        e_ang = np.abs(theta)
        u_ang = Kp_ang*e_ang

        if pose.theta > 0.08:
            msg.angular.z = u_ang*-1
        elif pose.theta < -0.08:
            msg.angular.z = u_ang*1
        else:
            msg.angular.z = 0.0

        # Linear Controller
        Kp_lin = 1.5 - np.sqrt(e_ang)
        Kd_lin = 0.01
        Tf = 0.1
        stop_pt = 0.30
        e_lin  = r - stop_pt
        # print(e_lin,'dist_error')
        u_lin = Kp_lin*e_lin + Kd_lin * (e_lin-self.e_lin_prev)/Tf
        # print(e_lin,u_lin, 'dist_error & linear_u')
        
        if np.abs(u_lin) > 0.15:
            if u_lin>0:
                msg.linear.x = 0.15
            else:
                msg.linear.x = -0.15
        elif np.abs(e_lin) > 0.025:
            msg.linear.x = u_lin
        else:
            msg.linear.x = 0.0

        self.e_lin_prev = e_lin

        print(msg.linear.x,'linear_vel')
        self._vel_publish.publish(msg)

def main():
    rclpy.init()
    move_robot = MoveRobot()

    while rclpy.ok():
        rclpy.spin_once(move_robot)
    
    move_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
