#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Camera viewer started. Waiting for camera data...')
        self.get_logger().info('Available topics:')
        
        # Create a timer to check for images
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.image_received = False
        
    def timer_callback(self):
        if not self.image_received:
            self.get_logger().info('Still waiting for camera images on topic /world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image...')
        
    def image_callback(self, msg):
        try:
            if not self.image_received:
                self.get_logger().info('Camera images received! Press "q" to quit.')
                self.image_received = True
                
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Add info overlay
            height, width = cv_image.shape[:2]
            cv2.putText(cv_image, f'Size: {width}x{height}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, 'Robot Camera Feed', (10, height - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display the image
            cv2.imshow('Robot Camera Feed', cv_image)
            
            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Shutting down camera viewer...')
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    camera_viewer = CameraViewer()
    
    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        camera_viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
