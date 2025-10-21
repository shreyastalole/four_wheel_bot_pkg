#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import threading
import queue
from concurrent.futures import ThreadPoolExecutor
import time

class CameraDataRecorder(Node):
    def __init__(self):
        super().__init__('camera_data_recorder')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Storage parameters
        self.declare_parameter('storage_path', '/home/jetracer/camera_data')
        self.declare_parameter('image_format', 'jpg')
        self.declare_parameter('compression_quality', 75)  # Lower for faster saves
        self.declare_parameter('max_queue_size', 100)
        self.declare_parameter('num_threads', 8)  # More threads for 10 FPS
        
        self.storage_path = self.get_parameter('storage_path').value
        self.image_format = self.get_parameter('image_format').value
        self.compression_quality = self.get_parameter('compression_quality').value
        self.max_queue_size = self.get_parameter('max_queue_size').value
        self.num_threads = self.get_parameter('num_threads').value
        
        # Create storage directory if it doesn't exist
        os.makedirs(self.storage_path, exist_ok=True)
        
        # Initialize variables
        self.sequence_counter = 0  # For maintaining order
        
        # Queue for ordered saving
        self.save_queue = queue.Queue(maxsize=self.max_queue_size)
        self.pending_saves = {}  # Dictionary to track pending saves by sequence number
        self.next_sequence_to_write = 0  # Next sequence number we're waiting to write
        self.save_lock = threading.Lock()
        
        # Set up compression parameters
        if self.image_format.lower() in ['jpg', 'jpeg']:
            self.compression_params = [cv2.IMWRITE_JPEG_QUALITY, self.compression_quality]
        elif self.image_format.lower() == 'png':
            self.compression_params = [cv2.IMWRITE_PNG_COMPRESSION, 3]
        else:
            self.compression_params = []
        
        # Thread pool for saving images
        self.thread_pool = ThreadPoolExecutor(max_workers=self.num_threads)
        
        # Start the ordered writer thread
        self.writer_thread = threading.Thread(target=self._ordered_writer, daemon=True)
        self.writer_thread.start()
        
        # Subscribe to camera topic
        self.camera_subscription = self.create_subscription(
            Image,
            '/world/road_track/model/ackermann_bot/link/base_link/sensor/camera_sensor/image',
            self.camera_callback,
            20  # Increased queue size for 10 FPS
        )
        
        # Statistics
        self.images_saved = 0
        self.images_dropped = 0
        self.frames_received = 0
        self.last_frame_time = time.time()
        
        # Stats timer (every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info(f'Camera data recorder started. Saving to: {self.storage_path}')
        self.get_logger().info(f'Using {self.num_threads} threads, saving EVERY frame received')
        
    def camera_callback(self, msg):
        """Callback function for camera data - saves EVERY frame"""
        self.frames_received += 1
        
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get ROS timestamp for synchronization
            current_time = self.get_clock().now()
            ros_timestamp_ns = current_time.nanoseconds
            
            # Get sequence number for this frame
            current_sequence = self.sequence_counter
            self.sequence_counter += 1
            
            # Create filename with ROS timestamp for synchronization
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            filename = f'frame_{current_sequence:08d}_{ros_timestamp_ns}_{timestamp}.{self.image_format}'
            filepath = os.path.join(self.storage_path, filename)
            
            # Submit to thread pool for async saving
            future = self.thread_pool.submit(
                self._save_image_worker, 
                cv_image.copy(), 
                current_sequence, 
                filename, 
                filepath
            )
            
            # Add callback to handle completion
            future.add_done_callback(lambda f: self._handle_save_completion(f))
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera frame: {str(e)}')
            self.images_dropped += 1
    
    def _save_image_worker(self, image_data, sequence_num, filename, filepath):
        """Worker function to save image and return result with sequence number"""
        try:
            success = cv2.imwrite(filepath, image_data, self.compression_params)
            return {
                'sequence': sequence_num,
                'filename': filename,
                'filepath': filepath,
                'success': success,
                'timestamp': time.time()
            }
        except Exception as e:
            return {
                'sequence': sequence_num,
                'filename': filename,
                'filepath': filepath,
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }
    
    def _ordered_writer(self):
        """Thread that ensures images are written in order"""
        while True:
            try:
                # Get completed save operation from queue
                result = self.save_queue.get(timeout=1.0)
                if result is None:  # Shutdown signal
                    break
                
                sequence_num = result['sequence']
                
                with self.save_lock:
                    # Store the result
                    self.pending_saves[sequence_num] = result
                    
                    # Write all consecutive sequences starting from next_sequence_to_write
                    while self.next_sequence_to_write in self.pending_saves:
                        save_result = self.pending_saves.pop(self.next_sequence_to_write)
                        
                        if save_result['success']:
                            self.images_saved += 1
                        else:
                            self.images_dropped += 1
                            error_msg = save_result.get('error', 'Unknown error')
                            self.get_logger().error(f"Failed to save {save_result['filename']}: {error_msg}")
                        
                        self.next_sequence_to_write += 1
                
            except queue.Empty:
                continue  # Timeout, check again
            except Exception as e:
                self.get_logger().error(f'Error in ordered writer: {str(e)}')
    
    def _handle_save_completion(self, future):
        """Handle completion of save operation"""
        try:
            result = future.result()
            # Put result in queue for ordered writing
            self.save_queue.put(result)
        except Exception as e:
            self.get_logger().error(f'Error handling save completion: {str(e)}')
    
    def print_stats(self):
        """Print saving statistics"""
        current_time = time.time()
        time_diff = current_time - self.last_frame_time
        fps_received = self.frames_received / time_diff if time_diff > 0 else 0
        
        total_attempts = self.images_saved + self.images_dropped
        success_rate = (self.images_saved / total_attempts * 100) if total_attempts > 0 else 0
        pending_count = len(self.pending_saves)
        
        self.get_logger().info(
            f'Stats: {self.frames_received} frames received ({fps_received:.1f} FPS), '
            f'{self.images_saved} saved, {self.images_dropped} dropped, '
            f'{success_rate:.1f}% success rate, {pending_count} pending'
        )
        
        # Reset counters for next interval
        self.frames_received = 0
        self.last_frame_time = current_time
    
    def shutdown(self):
        """Clean shutdown"""
        try:
            # Signal writer thread to stop
            self.save_queue.put(None)
            
            # Wait for thread to finish
            if self.writer_thread.is_alive():
                self.writer_thread.join(timeout=5.0)
            
            # Shutdown thread pool
            self.thread_pool.shutdown(wait=True)
            
        except Exception as e:
            # Don't use logger here as context might be invalid
            print(f'Error during shutdown: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    camera_recorder = None
    
    try:
        camera_recorder = CameraDataRecorder()
        rclpy.spin(camera_recorder)
    except KeyboardInterrupt:
        print('Camera data recorder shutting down...')  # Use print instead of logger
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        # Clean shutdown sequence
        if camera_recorder is not None:
            try:
                camera_recorder.shutdown()
            except Exception as e:
                print(f'Error during camera recorder shutdown: {str(e)}')
            
            try:
                camera_recorder.destroy_node()
            except Exception as e:
                print(f'Error destroying node: {str(e)}')
        
        # Only shutdown if ROS2 is still active
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f'Error during rclpy shutdown: {str(e)}')

if __name__ == '__main__':
    main()