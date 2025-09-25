#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Lidar listener node started')

    def lidar_callback(self, msg):
        # Process the laser scan data
        ranges = np.array(msg.ranges)
        
        # Filter out invalid readings (inf and nan)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            max_distance = np.max(valid_ranges)
            avg_distance = np.mean(valid_ranges)
            
            self.get_logger().info(
                f'Lidar data - Min: {min_distance:.2f}m, '
                f'Max: {max_distance:.2f}m, '
                f'Avg: {avg_distance:.2f}m, '
                f'Valid points: {len(valid_ranges)}/{len(ranges)}'
            )
        else:
            self.get_logger().warn('No valid lidar readings received')


def main(args=None):
    rclpy.init(args=args)
    
    lidar_listener = LidarListener()
    
    try:
        rclpy.spin(lidar_listener)
    except KeyboardInterrupt:
        pass
    
    lidar_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
