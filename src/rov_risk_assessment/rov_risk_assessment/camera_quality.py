#!/usr/bin/env python3
"""
Simple Camera Quality Node
Measures sharpness and brightness from camera image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraQualityNode(Node):
    def __init__(self):
        super().__init__('camera_quality_node')
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/blueye/camera_1/image_raw', self.image_callback, 10)
        
        # Publish camera quality
        self.quality_pub = self.create_publisher(String, '/blueye/camera_quality', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('✓ Camera Quality Node started')
    
    def image_callback(self, msg):
        """Analyze image and publish quality"""
        
        # Convert to grayscale
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 1. Sharpness (Laplacian variance - measures blur)
        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
        
        # 2. Brightness (average pixel value)
        brightness = np.mean(gray)
        
        # Determine quality based on thresholds
        if sharpness > 200 and 40 < brightness < 100:
            quality = "Excellent"
        elif sharpness > 50 and 20 < brightness < 80:
            quality = "Good"
        elif sharpness > 10:
            quality = "Poor"
        else:
            quality = "Failed"
        
        # Publish
        quality_msg = String()
        quality_msg.data = quality
        self.quality_pub.publish(quality_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraQualityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()