#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import os
from datetime import datetime
import threading
import time

class CommandRecorder(Node):
    def __init__(self):
        super().__init__('command_recorder')
        
        # Storage parameters
        self.declare_parameter('storage_path', '/home/jetracer/command_data')
        self.declare_parameter('csv_filename', 'robot_commands.csv')
        self.declare_parameter('record_zeros', False)  # Skip zero commands to reduce file size
        
        self.storage_path = self.get_parameter('storage_path').value
        self.csv_filename = self.get_parameter('csv_filename').value
        self.record_zeros = self.get_parameter('record_zeros').value
        
        # Create storage directory if it doesn't exist
        os.makedirs(self.storage_path, exist_ok=True)
        
        # CSV file setup
        self.csv_filepath = os.path.join(self.storage_path, self.csv_filename)
        self.csv_lock = threading.Lock()
        
        # Initialize CSV file with headers
        self._initialize_csv()
        
        # Subscribe to command topic
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.command_callback,
            10
        )
        
        # Statistics
        self.commands_recorded = 0
        self.commands_skipped = 0
        self.start_time = time.time()
        
        # Stats timer (every 10 seconds)
        self.stats_timer = self.create_timer(10.0, self.print_stats)
        
        self.get_logger().info(f'Command recorder started. Saving to: {self.csv_filepath}')
        self.get_logger().info(f'Recording zeros: {self.record_zeros}')
        
    def command_callback(self, msg):
        """Callback function for command data"""
        try:
            # Get ROS timestamp when command was received
            current_time = self.get_clock().now()
            ros_timestamp_ns = current_time.nanoseconds  # Single nanosecond timestamp
            
            # Check if we should skip zero commands
            if not self.record_zeros:
                if (msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.linear.z == 0.0 and
                    msg.angular.x == 0.0 and msg.angular.y == 0.0 and msg.angular.z == 0.0):
                    self.commands_skipped += 1
                    return
            
            # Human-readable timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # Record command data with unified timestamp
            command_data = [
                timestamp,
                ros_timestamp_ns,  # Single nanosecond timestamp for easy matching
                self.commands_recorded,
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            ]
            
            # Write to CSV (thread-safe)
            with self.csv_lock:
                with open(self.csv_filepath, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(command_data)
            
            self.commands_recorded += 1
            
        except Exception as e:
            self.get_logger().error(f'Error recording command: {str(e)}')
    
    def _initialize_csv(self):
        """Initialize CSV file with headers"""
        try:
            with open(self.csv_filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp',
                    'ros_timestamp_ns',  # Unified nanosecond timestamp
                    'sequence',
                    'linear_x',
                    'linear_y', 
                    'linear_z',
                    'angular_x',
                    'angular_y',
                    'angular_z'
                ])
            self.get_logger().info(f'Initialized CSV file: {self.csv_filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CSV file: {str(e)}')
    
    def print_stats(self):
        """Print recording statistics"""
        elapsed_time = time.time() - self.start_time
        total_commands = self.commands_recorded + self.commands_skipped
        
        self.get_logger().info(
            f'Command Stats: {self.commands_recorded} recorded, '
            f'{self.commands_skipped} skipped, '
            f'{total_commands} total in {elapsed_time:.1f}s'
        )
    
    def get_summary(self):
        """Get recording summary"""
        elapsed_time = time.time() - self.start_time
        return {
            'commands_recorded': self.commands_recorded,
            'commands_skipped': self.commands_skipped,
            'elapsed_time': elapsed_time,
            'csv_file': self.csv_filepath
        }

def main(args=None):
    rclpy.init(args=args)
    
    command_recorder = None
    
    try:
        command_recorder = CommandRecorder()
        rclpy.spin(command_recorder)
    except KeyboardInterrupt:
        print('Command recorder shutting down...')
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        if command_recorder is not None:
            # Print final summary
            summary = command_recorder.get_summary()
            print(f"\nRecording Summary:")
            print(f"Commands recorded: {summary['commands_recorded']}")
            print(f"Commands skipped: {summary['commands_skipped']}")
            print(f"Total time: {summary['elapsed_time']:.1f} seconds")
            print(f"CSV file: {summary['csv_file']}")
            
            try:
                command_recorder.destroy_node()
            except Exception as e:
                print(f'Error destroying node: {str(e)}')
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f'Error during rclpy shutdown: {str(e)}')

if __name__ == '__main__':
    main()