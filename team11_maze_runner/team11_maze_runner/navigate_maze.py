import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,PointStamped,Point
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage

import numpy as np
import math

class NavigateToGoal(Node):
    
    def __init__(self):

        self.curr_pose = PoseStamped()
        self.clicked_goal = PointStamped()
        self.set_goal = np.array([[1.9639177322387695,0.15323543548583984,-0.001434326171875],
                                  [1.0109432935714722,-1.731042146682739,0.0025329589843],
                                  [0.115954212844371, -0.854123294353,-0.001434326171]])
        self.goal_ind = 0
        self.num_goals = len(self.set_goal)
        self.USE_CLICKED_GOAL = True

        self.goal = PoseStamped()
        self.goal.header.stamp.sec = 0
        self.goal.header.stamp.nanosec = 0
        self.goal.header.frame_id = 'map'
        self.goal.pose.orientation.x = 0.0
        self.goal.pose.orientation.y = 0.0
        self.goal.pose.orientation.z = 0.0
        self.goal.pose.orientation.w = 1.0

        ## Test Waypoint
        

        super().__init__("navigate_to_goal")
        
        self._goal_subscriber = self.create_subscription(
                PointStamped,
                '/clicked_point',
                self._clicked_callback,
                10
        )
        self._pose_subscriber = self.create_subscription(
                PoseStamped,
                'NavigateToPose_FeedbackMessage',
                self._goal_callback,
                10
        )
        self._sign_subscriber = self.create_subscription(
                Point,
                '/sign',
                self._sign_callback,
                10
        )
        self._goal_publisher = self.create_publisher(
                PoseStamped,
                '/goal_pose',
                10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self._goal_callback)

    
    def _clicked_callback(self,clicked):
        self.clicked_goal = clicked
        # print(self.clicked_goal)

    def _pose_callback(self,pose):
        self.curr_pose = pose

    def _sign_callback(self,sign):
        self.sign_type = int(sign.x)

    def _goal_callback(self):
        msg = self.goal
        x = float()
        y = float()
        z = float()
        if self.USE_CLICKED_GOAL:
            x = self.clicked_goal.point.x
            y = self.clicked_goal.point.y
            z = self.clicked_goal.point.z
        else:
            x = self.set_goal[self.goal_ind][0]
            y = self.set_goal[self.goal_ind][1]
            z = self.set_goal[self.goal_ind][2]

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        goal_check_x = np.abs(x - self.curr_pose.pose.position.x) < 0.01
        goal_check_y = np.abs(y - self.curr_pose.pose.position.y) < 0.01
        print(msg)
        if goal_check_x and goal_check_y:
            if (not self.USE_CLICKED_GOAL) and (self.goal_ind < self.num_goals-1):
                self._goal_publisher.publish(msg)
                self.goal_ind += 1
        else:
            self._goal_publisher.publish(msg)


        

def main():
    rclpy.init()
    navigate_to_goal = NavigateToGoal()

    while rclpy.ok():
        rclpy.spin_once(navigate_to_goal)
    
    navigate_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
