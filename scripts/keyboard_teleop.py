#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 1.0    # m/s
        self.angular_speed = 1.0   # rad/s
        self.speed_increment = 0.1
        
        # Current twist message
        self.twist = Twist()
        
        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Keyboard Teleop Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/w: Forward')
        self.get_logger().info('  S/s: Backward') 
        self.get_logger().info('  A/a: Turn Left')
        self.get_logger().info('  D/d: Turn Right')
        self.get_logger().info('  Q/q: Increase Speed')
        self.get_logger().info('  E/e: Decrease Speed')
        self.get_logger().info('  Space: Stop')
        self.get_logger().info('  Ctrl+C: Exit')
        self.get_logger().info(f'Current linear speed: {self.linear_speed:.1f} m/s')
        self.get_logger().info(f'Current angular speed: {self.angular_speed:.1f} rad/s')

    def get_key(self, timeout=0.1):
        """Get a single keypress from the terminal with timeout"""
        if select.select([sys.stdin], [], [], timeout) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard input and publish twist message"""
        if key is None:
            return True
            
        # Handle Ctrl+C properly
        if ord(key) == 3:  # Ctrl+C
            self.get_logger().info('Ctrl+C pressed, exiting...')
            return False
        
        if key.lower() == 'w':
            # Forward
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0
            self.get_logger().info('Moving Forward')
            
        elif key.lower() == 's':
            # Backward
            self.twist.linear.x = -self.linear_speed
            self.twist.angular.z = 0.0
            self.get_logger().info('Moving Backward')
            
        elif key.lower() == 'a':
            # Turn left
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_speed
            self.get_logger().info('Turning Left')
            
        elif key.lower() == 'd':
            # Turn right
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.angular_speed
            self.get_logger().info('Turning Right')
            
        elif key.lower() == 'q':
            # Increase speed
            self.linear_speed += self.speed_increment
            self.angular_speed += self.speed_increment
            self.get_logger().info(f'Speed increased - Linear: {self.linear_speed:.1f}, Angular: {self.angular_speed:.1f}')
            return True  # Don't publish, just update speed
            
        elif key.lower() == 'e':
            # Decrease speed
            self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
            self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
            self.get_logger().info(f'Speed decreased - Linear: {self.linear_speed:.1f}, Angular: {self.angular_speed:.1f}')
            return True  # Don't publish, just update speed
            
        elif key == ' ':
            # Stop
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().info('Stopping')
            
        else:
            # Invalid key - stop the robot
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            
        # Publish the twist message
        self.publisher.publish(self.twist)
        return True

    def run(self):
        """Main loop for keyboard input"""
        # Set terminal to raw mode
        tty.setraw(sys.stdin.fileno())
        
        try:
            while rclpy.ok():
                # Spin ROS once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.0)
                
                # Get key with timeout
                key = self.get_key(timeout=0.1)
                
                if not self.process_key(key):
                    break
                    
        except KeyboardInterrupt:
            self.get_logger().info('KeyboardInterrupt caught, exiting...')
        finally:
            # Stop the robot
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)
    
    teleop_node = KeyboardTeleop()
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
