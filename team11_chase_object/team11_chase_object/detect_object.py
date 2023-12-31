#!/usr/bin/python
# Team 11
# Haris Miller and Atharva Mete

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class ObjectTracker(Node):

    def __init__(self):
        # Creates the node.
        super().__init__('object_tracker')
		# Set Parameters
        self.declare_parameter('show_image_bool', False)
        self.declare_parameter('window_name', "Raw Image")
		
		#Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		
		#Only create image frames if we are not running headless (_display_image sets this)
        if(self._display_image):
		# Set Up Image Viewing
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
            cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
		
		#Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

		#Declare that the image_subscriber node is subcribing to the /camera/image/compressed topic.
        self._image_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
        self._point_publisher = self.create_publisher(
                Point,
                'objCenter',
                10)
        self._image_subscriber # Prevents unused variable warning.
        self._point_publisher

    def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        
        
        height, width = self._imgBGR.shape[:2]
        # print(height,width)
        # Our operations on the self._imgBGR come here
        hsv = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([48, 69, 71])
        higher_hsv = np.array([145, 212, 214])
        # Apply the cv2.inrange method to create a mask
        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
        mask = cv2.GaussianBlur(mask,(5,5),0)

        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 200, param1=50, param2=20, minRadius=15, maxRadius=100)
        # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 200, param1=50, param2=20,minRadius=20, maxRadius=800)
        msg = Point()
        if not (circles is None):
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # print(i[0]- width/2,i[1])
                msg.x = float(i[0] - width/2)
                msg.y = float(i[1] - height/2)
                cv2.circle(self._imgBGR,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(self._imgBGR,(i[0],i[1]),2,(0,0,255),3)
                # print(i[2],'radius')
                self._point_publisher.publish(msg)
        if(self._display_image):
			# Display the image in a window
            self.show_image(self._imgBGR)

    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
        self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

    def get_user_input(self):
        return self._user_input

def main():
    rclpy.init()
    object_tracker = ObjectTracker()

    while rclpy.ok():
        rclpy.spin_once(object_tracker)
    
    object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
      main()
