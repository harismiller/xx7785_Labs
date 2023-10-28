import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,Twist

import numpy as np
import math

class MoveRobot(Node):

    def __init__(self):

        #Variables
        self.odom = Pose2D()
        self.waypts = np.array([[3, 0]])
        self.dstar = 5
        self.qstar = 5
        self.zeta = 1
        self.neta = 1
        self.reached_threshold = 0.05
        self.current_wp_index = 0

        self.MAX_LIN = 0.2
        self.MAX_ANG = 1.5

        super().__init__('move_robot')

        self._odom_subcriber = self.create_subscription(
                Pose2D,
                '/odomUpdate',
                self._odom_callback,
                10)
        
        self._obj_subcriber = self.create_subscription(
                Pose2D,
                '/obsLocation',
                self._move_callback,
                10)

        self._move_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                5)
        
    def _move_callback(self, point):
        msg = Twist()
        if self.current_wp_index > len(self.waypts):
            goal = self.waypts[self.current_wp_index]
            #determine waypoint based on your algo
            curr_pos = [self.odom.x,self.odom.y]
            dist_to_goal = np.sqrt((curr_pos[0]-goal[0])**2+(curr_pos[1]-goal[1])**2)
            if dist_to_goal>self.dstar:
                xa,ya = -self.dstar*self.zeta*(curr_pos-goal)/dist_to_goal
            else:
                xa,ya = -self.zeta*(np.array(curr_pos)-np.array(goal))
            xr = []
            yr = []
            dist_to_obstacle = point.z
            closest_point = [point.x,point.y]
            if dist_to_obstacle<self.qstar:
                # print('ons_in_con')
                coeff = -self.neta*(dist_to_obstacle-self.qstar)/((dist_to_obstacle**4)*self.qstar)
                # print(coeff,'coef')
                xrg,yrg = coeff*(np.array(curr_pos)-np.array(closest_point))
            else:
                xrg,yrg = 0,0
            xr.append(xrg)
            yr.append(yrg)
            xr = np.array(xr)
            yr = np.array(yr)
            print(xa,ya,'attraction')
            print(xr,yr,'repultion')
            xut = xa+np.sum(xr)
            yut = ya+np.sum(yr)
            xu = xut/np.sqrt(xut**2+yut**2)
            yu = yut/np.sqrt(xut**2+yut**2)

            xpos = curr_pos[0]
            ypos = curr_pos[1]
            xu = xu*self.MAX_LIN
            yu = yu*self.MAX_LIN
            lin_vel = (xpos*xu + ypos*yu)/np.sqrt(xpos**2 + ypos**2)
            msg.linear.x = lin_vel
            # ang_vel = math.atan2(yu,xu)
            ang_vel = (xpos*yu - ypos*xu)/(xpos**2 + ypos**2)
            if np.abs(ang_vel)>self.MAX_ANG:
                if ang_vel < 0:
                    ang_vel = -self.MAX_ANG
                else:
                    ang_vel = self.MAX_ANG
            msg.angular.z = ang_vel
            print(msg.linear.x,msg.angular.z,'linear-angular')
            if dist_to_goal<self.reached_threshold:
                self.current_wp_index+=1
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

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
