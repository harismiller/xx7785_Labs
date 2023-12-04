import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
import os
import torch
from torchvision import models, transforms
import torch.nn as nn
from PIL import Image

class DetectSign(Node):
    
    def __init__(self):
        super().__init__('detect_sign')
        self.preprocess = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        model_path = "/home/atharva/courses/irr_ws/src/xx7785_Lab2/team11_maze_runner/team11_maze_runner/fine_tuned_shufflenet.pth"
        self.net = models.shufflenet_v2_x0_5(pretrained=False)
        num_classes = 6
        # Replace the last layer with a new Linear layer
        self.net.fc = nn.Linear(in_features=self.net.fc.in_features, out_features=num_classes)
        # Load the fine-tuned model weights
        self.net.load_state_dict(torch.load(model_path,map_location=torch.device('cpu')))
        # self.net.to('cpu')
        self.net.eval()

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
        imgRGB = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "rgb8")
        # imgRGB = np.asarray(imgRGB)
        imgRGB = Image.fromarray(imgRGB)
        # imgRGB = torch.from_numpy(imgRGB)
        # print(type(imgRGB))
        input_tensor = self.preprocess(imgRGB)
        # create a mini-batch as expected by the model
        input_batch = input_tensor.unsqueeze(0)
        # run model
        output = self.net(input_batch)
        sign_type = int(torch.argmax(output))
        print(sign_type)
        msg = Point()
        msg.x = float(sign_type)
        self._sign_publisher.publish(msg)
        

def main():
    rclpy.init()
    detect_sign = DetectSign()

    while rclpy.ok():
        rclpy.spin_once(detect_sign)
    
    detect_sign.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
      main()
