#!/usr/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,Twist,Point

import numpy as np

class MoveRobot(Node):

    def __init__(self):

        # Variables
        self.e_lin = 0.3
        self.e_lin_prev = 0
        self.target_orient_flag = False

        super().__init__('move_robot')

        self._pose_subscriber = self.create_subscription(
                Pose2D,
                '/wallLocation',
                self._chase_callback,
                10)
        self._odom_subscriber = self.create_subscription(
            Pose2D,
            '/odomUpdate',
            self._odom_callback,
            10
        )
        self._sign_subscriber = self.create_subscription(
            Point,
            '/sign',
            self._sign_callback,
            10
        )
        self._vel_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                5)

    def _sign_callback(self,sign):
        self.sign_type = int(sign.x)
        if self.sign_type == 0:
            self.sign_value = 0
        elif self.sign_type == 1:
            self.sign_value = np.deg2rad(90)
        elif self.sign_type == 2:
            self.sign_value = -np.deg2rad(90)
        elif self.sign_type == 3 or self.sign_type == 4:
            self.sign_value = np.deg2rad(180)
        else:
            self.sign_value = 0

    def _odom_callback(self,pose):
        self.robot_pose = pose

    def _chase_callback(self, pose):
        msg = Twist()
        r = pose.x
        # print(r,'goal_dist')
        theta = pose.theta

        # Controller Gains
        Kp_ang = 0.45
        Kd_lin = 0.01
        Tf = 0.1

        # Switch linear and angular modes
        stop_pt = 0.40
        
        if self.e_lin < 0.01:
            print('turning only')
            if self.target_orient_flag:
                target_orient = self.robot_pose.theta+self.sign_value
            e_ang = target_orient-self.robot_pose.theta
            u_ang = Kp_ang*e_ang

            # if pose.theta > 0.03:
            #     msg.angular.z = u_ang*-1
            # elif pose.theta < -0.03:
            #     msg.angular.z = u_ang*1
            # self.target_orient_flag = False
            msg.linear.x = 0.0
            msg.angular.z = u_ang
            if np.abs(e_ang)<0.034:
                self.e_lin  = r - stop_pt
        else:
            print('moving ahead')
            self.e_lin  = r - stop_pt
            self.target_orient_flag = True
            e_ang = np.abs(theta)
            u_ang = Kp_ang*e_ang

            if pose.theta > 0.03:
                msg.angular.z = u_ang*-1
            elif pose.theta < -0.03:
                msg.angular.z = u_ang*1
            else:
                msg.angular.z = 0.0
            Kp_lin = 1.5 - np.sqrt(e_ang)
            u_lin = Kp_lin*self.e_lin + Kd_lin * (self.e_lin-self.e_lin_prev)/Tf
            
            if np.abs(u_lin) > 0.15:
                if u_lin>0:
                    msg.linear.x = 0.15
                else:
                    msg.linear.x = -0.15
            else:
                msg.linear.x = u_lin

        self.e_lin_prev = self.e_lin
        print(self.e_lin,e_ang,'errors')
        # print(msg.linear.z,'linear_vel')
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
