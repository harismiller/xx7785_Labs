import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class DetectSign(Node):
    
    def __init__(self):
        super().__init__('detect_sign')

        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self._image_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
        self._sign_publisher = self.create_publisher(
                Point,
                '/sign',
                10)
        self._image_subscriber # Prevents unused variable warning.
        self._sign_publisher

    def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        
        sign_type = 0
        msg = Point()
        msg.x = sign_type

def main():
    rclpy.init()
    detect_sign = DetectSign()

    while rclpy.ok():
        rclpy.spin_once(detect_sign)
    
    detect_sign.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
      main()
