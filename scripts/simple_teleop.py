#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys


class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        
        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 1.0    # m/s
        self.angular_speed = 1.0   # rad/s
        
        self.get_logger().info('Simple Teleop Node Started')
        self.get_logger().info('Enter commands:')
        self.get_logger().info('  w: Forward    s: Backward')
        self.get_logger().info('  a: Left       d: Right')
        self.get_logger().info('  x: Stop       q: Quit')

    def run(self):
        """Simple input loop"""
        try:
            while rclpy.ok():
                try:
                    # Get input from user
                    key = input("Command (w/a/s/d/x/q): ").strip().lower()
                    
                    twist = Twist()
                    
                    if key == 'w':
                        twist.linear.x = self.linear_speed
                        self.get_logger().info('Moving Forward')
                    elif key == 's':
                        twist.linear.x = -self.linear_speed
                        self.get_logger().info('Moving Backward')
                    elif key == 'a':
                        twist.angular.z = self.angular_speed
                        self.get_logger().info('Turning Left')
                    elif key == 'd':
                        twist.angular.z = -self.angular_speed
                        self.get_logger().info('Turning Right')
                    elif key == 'x':
                        # Stop
                        self.get_logger().info('Stopping')
                    elif key == 'q':
                        self.get_logger().info('Quitting...')
                        break
                    else:
                        self.get_logger().info('Unknown command, stopping')
                    
                    # Publish the command
                    self.publisher.publish(twist)
                    
                except EOFError:
                    break
                except KeyboardInterrupt:
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            # Send stop command
            stop_twist = Twist()
            self.publisher.publish(stop_twist)
            self.get_logger().info('Sent stop command')


def main(args=None):
    rclpy.init(args=args)
    
    teleop_node = SimpleTeleop()
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
