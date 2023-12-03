import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D,Twist, Point

import numpy as np
import math
import time

class MoveRobot(Node):

    def __init__(self):
        # time.sleep(2)
        #Variables
        self.odom = Pose2D()
        # self.waypts = np.array([[1.7, 0]])
        self.dstar = 0.15 # 0.2
        self.qstar = 0.3# 0.4
        self.zeta = 1 # 1
        self.neta = 0.01 # 0.01
        self.reached_threshold = 0.05
        # self.current_wp_index = 0
        self.goal_add = [0.0,0.0]
        self.ang_thresh_active = False
        self.turn_goal_thresh = False
        self.turn_thresh = False
        self.dist_check = 0.0

        self.MAX_LIN = 0.2
        self.MAX_ANG = 1.5
        self.ANGLE_THRESHOLD = math.radians(170)
        self.MIN_DIST = 0.5
        # self.MIN_DIST_ANG_THRESH = 0.4
        # self.wait_count = 20

        super().__init__('move_robot')

        self._odom_subcriber = self.create_subscription(
                Pose2D,
                '/odomUpdate',
                self._odom_callback,
                10)
        
        self._obj_subcriber = self.create_subscription(
                Pose2D,
                '/wallLocation',
                self._move_callback,
                10)
        self._sign_subscriber = self.create_subscription(
            Point,
            '/sign',
            self._sign_callback,
            10
        )
        self._move_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                10)
        
    def normalize_vector(self,vector):
        return vector / np.linalg.norm(vector)

    def get_angle(self,xa,ya,xr,yr):
        v1 = np.array([xa,ya])
        v2 = np.array([xr,yr])
        v1_n = self.normalize_vector(v1)
        v2_n = self.normalize_vector(v2)
        return np.arccos(np.clip(np.dot(v1_n, v2_n), -1.0, 1.0)),v1_n,v2_n
    
    def transform_to_global(self,vec,orient):
        local_to_global = np.array([[np.cos(orient),-np.sin(orient)],
                           [np.sin(orient), np.cos(orient)]])
        return local_to_global.dot(vec)
        
    def _sign_callback(self,sign):
        self.sign_type = int(sign.x)
        if self.sign_type == 0:
            self.sign_value = [0,-0.1]
        elif self.sign_type == 1:
            self.sign_value = [0.0,0.1]
        elif self.sign_type == 2:
            self.sign_value = [0.0,-0.1]
        elif self.sign_type == 3 or self.sign_type == 4:
            self.sign_value = [-0.5,0.0]
        else:
            self.sign_value = [0.0,0.0]
    
    def _move_callback(self, point):
        msg = Twist()
        #determine waypoint based on your algo
        curr_pos = [self.odom.x,self.odom.y]
        orientation = self.odom.theta

        dist_to_obstacle = point.x
        angle_to_obstacle = point.theta

        closest_point_local = np.array([dist_to_obstacle*np.cos(angle_to_obstacle),
                                        dist_to_obstacle*np.sin(angle_to_obstacle)])
        closest_point_off = self.transform_to_global(closest_point_local,orientation)
        closest_point = np.array([curr_pos[0]+closest_point_off[0],curr_pos[1]+closest_point_off[1]])

        e_lin = dist_to_obstacle - self.MIN_DIST
        if e_lin < 0.01:
            print('check sign')
            if self.turn_goal_thresh==False:
                self.goal_add = np.array(self.sign_value)
            self.turn_goal_thresh = True
            self.turn_thresh = True
        elif self.turn_thresh:
            print('re-orienting')
            if self.dist_check < 0.03:
                self.turn_thresh = False
        else:
            self.turn_goal_thresh = False
            self.goal_add = np.array([0.1,0.0])

        goal_add_transform = self.transform_to_global(self.goal_add,orientation)
        goal = [curr_pos[0]+goal_add_transform[0],curr_pos[1]+goal_add_transform[1]]
        dist_to_goal = np.sqrt((curr_pos[0]-goal[0])**2+(curr_pos[1]-goal[1])**2)
        self.dist_check = dist_to_goal
        if dist_to_goal>self.dstar:
            xa,ya = -self.dstar*self.zeta*(np.array(curr_pos)-np.array(goal))/dist_to_goal
        else:
            xa,ya = -self.zeta*(np.array(curr_pos)-np.array(goal))
        xr = []
        yr = []
        

        if dist_to_obstacle<self.qstar:
            # print('ons_in_con')
            coeff = -self.neta*(dist_to_obstacle-self.qstar)/((dist_to_obstacle**4)*self.qstar)
            # print(coeff,'coef')
            xrg,yrg = coeff*(np.array(curr_pos)-closest_point)
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
        angle,a_n,r_n = self.get_angle(xa,ya,xr[0],yr[0])
        # print(a_n,r_n)
        # if dist_to_obstacle<self.MIN_DIST_ANG_THRESH:
        if angle>self.ANGLE_THRESHOLD:
            print('angle_thresh active')
            if self.ang_thresh_active==False:
                net_vec = np.add(a_n,r_n)
                # print(net_vec)
                self.net_vec_n = self.normalize_vector(net_vec)
            xu = self.net_vec_n[0]
            yu = self.net_vec_n[1]
            self.ang_thresh_active = True
        else:
            self.ang_thresh_active = False

        print(xu,yu,'xuyu_before')
        lin_vel = self.MAX_LIN*(xu*np.cos(orientation)+yu*np.sin(orientation))
        ang_vel = self.MAX_ANG*(yu*np.cos(orientation)-xu*np.sin(orientation))
        # if lin_vel<0.0:
            # lin_vel=0.0
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        print(msg.linear.x,msg.angular.z,'linear-angular')
        # if dist_to_goal<self.reached_threshold:
        #     print('wp_changed_yayayayayayayayayayayayayayayayayayayayayayayayayayayayayayayaya')
        #     self.current_wp_index+=1
        #     self.wait_count = 5

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
