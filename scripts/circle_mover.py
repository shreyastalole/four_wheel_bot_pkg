#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        
        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer to publish commands regularly
        self.timer = self.create_timer(0.1, self.move_in_circle)  # 10 Hz
        
        self.get_logger().info('Circle Mover Node Started - Robot will move in clockwise circle')
        self.get_logger().info('Press Ctrl+C to stop')

    def move_in_circle(self):
        """Make the robot move in a clockwise circle"""
        twist = Twist()
        
        # For clockwise circle:
        # - Positive linear velocity (forward)
        # - Negative angular velocity (turn right)
        twist.linear.x = 0.5   # Forward speed
        twist.angular.z = -0.3  # Turn right (negative for clockwise)
        
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    circle_mover = CircleMover()
    
    try:
        rclpy.spin(circle_mover)
    except KeyboardInterrupt:
        # Stop the robot
        stop_twist = Twist()
        circle_mover.publisher.publish(stop_twist)
        circle_mover.get_logger().info('Stopping robot...')
    finally:
        circle_mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
