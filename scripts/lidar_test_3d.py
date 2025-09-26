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
        
        # Add counters for diagnostics
        self.message_count = 0
        self.invalid_message_count = 0

    def lidar_callback(self, msg):
        self.message_count += 1
        
        # Process the laser scan data
        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities)
        
        # Check for the -.inf problem you encountered
        negative_inf_count = np.sum(ranges == float('-inf'))
        positive_inf_count = np.sum(ranges == float('inf'))
        nan_count = np.sum(np.isnan(ranges))
        
        # Filter out invalid readings (inf and nan)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            max_distance = np.max(valid_ranges)
            avg_distance = np.mean(valid_ranges)
            
            # Process intensity data
            if len(intensities) > 0 and len(intensities) == len(ranges):
                valid_intensities = intensities[np.isfinite(ranges)]
                if len(valid_intensities) > 0:
                    min_intensity = np.min(valid_intensities)
                    max_intensity = np.max(valid_intensities)
                    avg_intensity = np.mean(valid_intensities)
                    
                    self.get_logger().info(
                        f'Msg #{self.message_count} - Distance: Min={min_distance:.2f}m, Max={max_distance:.2f}m, Avg={avg_distance:.2f}m | '
                        f'Intensity: Min={min_intensity:.1f}, Max={max_intensity:.1f}, Avg={avg_intensity:.1f} | '
                        f'Valid: {len(valid_ranges)}/{len(ranges)} | -inf: {negative_inf_count}, +inf: {positive_inf_count}, nan: {nan_count}'
                    )
                else:
                    self.get_logger().info(
                        f'Msg #{self.message_count} - Distance: Min={min_distance:.2f}m, Max={max_distance:.2f}m, Avg={avg_distance:.2f}m | '
                        f'Intensity: No valid data | Valid: {len(valid_ranges)}/{len(ranges)} | -inf: {negative_inf_count}, +inf: {positive_inf_count}, nan: {nan_count}'
                    )
            else:
                self.get_logger().info(
                    f'Msg #{self.message_count} - Distance: Min={min_distance:.2f}m, Max={max_distance:.2f}m, Avg={avg_distance:.2f}m | '
                    f'Intensity: Not available | Valid: {len(valid_ranges)}/{len(ranges)} | -inf: {negative_inf_count}, +inf: {positive_inf_count}, nan: {nan_count}'
                )
        else:
            self.invalid_message_count += 1
            self.get_logger().warn(
                f'Msg #{self.message_count} - No valid lidar readings! '
                f'Total points: {len(ranges)} | -inf: {negative_inf_count}, +inf: {positive_inf_count}, nan: {nan_count} | '
                f'Invalid messages so far: {self.invalid_message_count}/{self.message_count}'
            )
            
            # Log detailed info for debugging when all readings are invalid
            if self.message_count <= 5:  # Only for first few messages to avoid spam
                self.get_logger().info(f'Range limits: min={msg.range_min}, max={msg.range_max}')
                self.get_logger().info(f'Angle range: {msg.angle_min:.3f} to {msg.angle_max:.3f} rad')
                self.get_logger().info(f'Frame ID: {msg.header.frame_id}')


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