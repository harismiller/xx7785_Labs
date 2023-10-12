import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D,Twist

import numpy as np

class RobotRotation(Node):

    def __init__(self):

        # Variables
        self.e_lin_prev = 0

        super().__init__('robot_rotation')

        self._pose_subscriber = self.create_subscription(
                Pose2D,
                '/objLocation',
                self._chase_callback,
                10)
        self._vel_publish = self.create_publisher(
                Twist,
                '/cmd_vel',
                5)
    
    def _chase_callback(self, pose):
        msg = Twist()
        r = pose.x
        theta = pose.theta

        # Angular Controller
        Kp_ang = 1
        e_ang = np.abs(theta)
        u_ang = Kp_ang*e_ang

        if pose.theta > 0.08:
            msg.angular.z = u_ang*-1
        elif pose.theta < -0.08:
            msg.angular.z = u_ang*1
        else:
            msg.angular.x = 0.0

        # Linear Controller
        Kp_lin = 1 - np.sqrt(e_ang)
        Kd_lin = 0.01
        Tf = 0.1
        stop_pt = 0.1
        e_lin  = np.abs(stop_pt - r)
        u_lin = Kp_lin*e_lin + Kd_lin * (e_lin-self.e_lin_prev)/Tf
        
        if u_lin > 0.12:
            msg.linear.x = 0.12
        else:
            msg.linear.x = u_lin

        self.e_lin_prev = e_lin

        print(msg)
        self._vel_publish.publish(msg)

def main():
    rclpy.init()
    robot_rotation = RobotRotation()

    while rclpy.ok():
        rclpy.spin_once(robot_rotation)
    
    robot_rotation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
