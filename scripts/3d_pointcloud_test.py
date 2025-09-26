#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2


class PointCloudListener(Node):
    def __init__(self):
        super().__init__('pointcloud_listener')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan/points',
            self.pointcloud_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('PointCloud listener node started')
        
        self.message_count = 0

    def pointcloud_callback(self, msg):
        self.message_count += 1
        
        try:
            # Extract point cloud data
            points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=False))
            
            if len(points) > 0:
                # Convert to numpy arrays for analysis
                xyz_data = np.array([(p[0], p[1], p[2]) for p in points])
                intensity_data = np.array([p[3] for p in points])
                ring_data = np.array([p[4] for p in points])
                
                # Filter out invalid points
                valid_mask = np.isfinite(xyz_data).all(axis=1)
                valid_points = xyz_data[valid_mask]
                valid_intensities = intensity_data[valid_mask]
                valid_rings = ring_data[valid_mask]
                
                if len(valid_points) > 0:
                    # Calculate statistics
                    distances = np.linalg.norm(valid_points, axis=1)
                    min_dist = np.min(distances)
                    max_dist = np.max(distances)
                    avg_dist = np.mean(distances)
                    
                    # Z-coordinate statistics (height information)
                    z_coords = valid_points[:, 2]
                    min_z = np.min(z_coords)
                    max_z = np.max(z_coords)
                    
                    # Intensity statistics
                    if len(valid_intensities) > 0 and not np.all(valid_intensities == 0):
                        min_intensity = np.min(valid_intensities)
                        max_intensity = np.max(valid_intensities)
                        avg_intensity = np.mean(valid_intensities)
                        intensity_info = f"Min={min_intensity:.1f}, Max={max_intensity:.1f}, Avg={avg_intensity:.1f}"
                    else:
                        intensity_info = "All zeros or invalid"
                    
                    # Ring statistics (vertical layers)
                    unique_rings = np.unique(valid_rings)
                    num_rings = len(unique_rings)
                    
                    self.get_logger().info(
                        f'PointCloud #{self.message_count} - Points: {len(valid_points)}/{len(points)} valid | '
                        f'Distance: {min_dist:.2f}-{max_dist:.2f}m (avg={avg_dist:.2f}) | '
                        f'Height: {min_z:.2f}-{max_z:.2f}m | '
                        f'Intensity: {intensity_info} | '
                        f'Rings: {num_rings} layers | '
                        f'Dimensions: {msg.width}x{msg.height}'
                    )
                    
                    # Check if it's truly 3D
                    if msg.height > 1:
                        self.get_logger().info(f'✓ 3D LiDAR confirmed: {msg.height} vertical layers')
                    else:
                        self.get_logger().warn(f'⚠ Appears to be 2D LiDAR: only {msg.height} layer')
                        
                else:
                    self.get_logger().warn(f'PointCloud #{self.message_count} - No valid points found!')
            else:
                self.get_logger().warn(f'PointCloud #{self.message_count} - Empty point cloud received!')
                
        except Exception as e:
            self.get_logger().error(f'Error processing PointCloud #{self.message_count}: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    pointcloud_listener = PointCloudListener()
    
    try:
        rclpy.spin(pointcloud_listener)
    except KeyboardInterrupt:
        pass
    
    pointcloud_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()