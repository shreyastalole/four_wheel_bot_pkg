#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class LidarTopicChecker(Node):
    def __init__(self):
        super().__init__('lidar_topic_checker')
        
        # Subscribe to LaserScan topic
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan',
            self.laser_callback,
            10)
        
        # Subscribe to PointCloud2 topic
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan/points',
            self.pointcloud_callback,
            10)
        
        self.laser_msg_count = 0
        self.pointcloud_msg_count = 0
        
        # Create a timer to print status
        self.timer = self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('Lidar topic checker started')
        self.get_logger().info('Listening for LaserScan and PointCloud2 messages...')

    def laser_callback(self, msg):
        self.laser_msg_count += 1
        if self.laser_msg_count % 10 == 0:  # Print every 10th message
            self.get_logger().info(f'LaserScan received #{self.laser_msg_count} - Range count: {len(msg.ranges)}')

    def pointcloud_callback(self, msg):
        self.pointcloud_msg_count += 1
        if self.pointcloud_msg_count % 10 == 0:  # Print every 10th message
            self.get_logger().info(f'PointCloud2 received #{self.pointcloud_msg_count} - Points: {msg.width * msg.height}')

    def print_status(self):
        self.get_logger().info(
            f'Status: LaserScan messages: {self.laser_msg_count}, '
            f'PointCloud2 messages: {self.pointcloud_msg_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    checker = LidarTopicChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    
    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
