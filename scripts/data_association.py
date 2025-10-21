#!/usr/bin/env python3

import pandas as pd
import os
import glob
import re
from datetime import datetime

def associate_data(camera_dir, command_csv, output_csv, time_tolerance_ms=100):
    """
    Associate camera images with command data based on timestamps
    
    Args:
        camera_dir: Directory containing camera images
        command_csv: Path to command CSV file
        output_csv: Output CSV with associations
        time_tolerance_ms: Maximum time difference in milliseconds for association
    """
    
    # Load command data
    commands_df = pd.read_csv(command_csv)
    
    # Extract timestamps from image filenames
    image_files = glob.glob(os.path.join(camera_dir, "*.jpg"))
    image_data = []
    
    for img_path in image_files:
        filename = os.path.basename(img_path)
        # Extract ROS timestamp from filename: frame_XXXXXXXX_TIMESTAMP_datetime.jpg
        match = re.search(r'frame_(\d+)_(\d+)_', filename)
        if match:
            sequence = int(match.group(1))
            ros_timestamp_ns = int(match.group(2))
            image_data.append({
                'image_path': img_path,
                'image_filename': filename,
                'sequence': sequence,
                'ros_timestamp_ns': ros_timestamp_ns
            })
    
    images_df = pd.DataFrame(image_data)
    
    # Associate images with commands
    associations = []
    time_tolerance_ns = time_tolerance_ms * 1_000_000  # Convert to nanoseconds
    
    for _, img_row in images_df.iterrows():
        img_time = img_row['ros_timestamp_ns']
        
        # Find closest command within tolerance
        time_diffs = abs(commands_df['ros_timestamp_ns'] - img_time)
        closest_idx = time_diffs.idxmin()
        closest_time_diff = time_diffs.iloc[closest_idx]
        
        if closest_time_diff <= time_tolerance_ns:
            cmd_row = commands_df.iloc[closest_idx]
            associations.append({
                'image_filename': img_row['image_filename'],
                'image_path': img_row['image_path'],
                'image_sequence': img_row['sequence'],
                'image_timestamp_ns': img_row['ros_timestamp_ns'],
                'command_sequence': cmd_row['sequence'],
                'command_timestamp_ns': cmd_row['ros_timestamp_ns'],
                'time_diff_ms': closest_time_diff / 1_000_000,
                'linear_x': cmd_row['linear_x'],
                'linear_y': cmd_row['linear_y'],
                'linear_z': cmd_row['linear_z'],
                'angular_x': cmd_row['angular_x'],
                'angular_y': cmd_row['angular_y'],
                'angular_z': cmd_row['angular_z']
            })
    
    # Save associations
    associations_df = pd.DataFrame(associations)
    associations_df.to_csv(output_csv, index=False)
    
    print(f"Associated {len(associations)} images with commands")
    print(f"Average time difference: {associations_df['time_diff_ms'].mean():.2f} ms")
    print(f"Results saved to: {output_csv}")
    
    return associations_df

if __name__ == '__main__':
    # Example usage
    camera_dir = '/home/jetracer/camera_data'
    command_csv = '/home/jetracer/command_data/robot_commands.csv'
    output_csv = '/home/jetracer/associated_data.csv'
    
    associate_data(camera_dir, command_csv, output_csv)