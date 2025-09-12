#!/usr/bin/env python3

"""
Demo script showing usage of both robot types
"""

import os
import sys

def print_header():
    print("="*60)
    print("    FOUR WHEEL BOT PACKAGE - ROS2 JAZZY + GAZEBO HARMONIC")
    print("="*60)
    print()

def print_robot_info():
    print("🤖 Available Robot Types:")
    print()
    print("1. DIFFERENTIAL DRIVE BOT (Blue)")
    print("   - Tank-style movement")
    print("   - Independent left/right wheel control")
    print("   - Best for: Precise maneuvering, spot turns")
    print("   - All 4 wheels are driven")
    print()
    print("2. ACKERMANN STEERING BOT (Red)")
    print("   - Car-style movement") 
    print("   - Front wheels steer, rear wheels drive")
    print("   - Best for: Smooth turns, highway-style navigation")
    print("   - Front 2 wheels steer, rear 2 wheels drive")
    print()

def print_controls():
    print("🎮 Control Options:")
    print()
    print("1. SIMPLE TELEOP (Recommended):")
    print("   Enter commands in terminal:")
    print("   w: Forward    s: Backward")
    print("   a: Left       d: Right")
    print("   x: Stop       q: Quit")
    print()
    print("2. KEYBOARD TELEOP (Alternative):")
    print("   Real-time WASD controls:")
    print("   W/w: Forward   S/s: Backward")
    print("   A/a: Left      D/d: Right")
    print("   Q/q: Speed Up  E/e: Speed Down")
    print("   Space: Stop    Ctrl+C: Exit")
    print()

def print_commands():
    print("🚀 Quick Start Commands:")
    print()
    print("# Build the package:")
    print("cd ~/ros2_ws")
    print("colcon build --packages-select four_wheel_bot_pkg")
    print("source install/setup.bash")
    print()
    print("# Launch Differential Drive Robot (recommended):")
    print("ros2 launch four_wheel_bot_pkg teleop_bot.launch.py")
    print()
    print("# Launch Ackermann Steering Robot:")
    print("ros2 launch four_wheel_bot_pkg teleop_bot.launch.py robot_type:=ackermann")
    print()
    print("# Manual launch (two terminals):")
    print("# Terminal 1:")
    print("ros2 launch four_wheel_bot_pkg diff_drive_bot.launch.py")
    print("# Terminal 2:")
    print("ros2 run four_wheel_bot_pkg simple_teleop.py")
    print()
    print("# Alternative keyboard teleop:")
    print("ros2 run four_wheel_bot_pkg keyboard_teleop.py")
    print()

def print_features():
    print("✨ Features:")
    print()
    print("✓ ROS2 Jazzy compatible")
    print("✓ Gazebo Harmonic compatible")
    print("✓ Two distinct robot types")
    print("✓ WASD keyboard control")
    print("✓ Realistic physics simulation")
    print("✓ Proper URDF with inertias")
    print("✓ Modern gz-sim plugins")
    print("✓ Clean, readable code")
    print("✓ Minimal dependencies")
    print("✓ Easy to extend")
    print()

def main():
    print_header()
    print_robot_info()
    print_controls()
    print_commands()
    print_features()
    
    print("📖 For more information, see README.md")
    print("🐛 For issues, check the troubleshooting section in README.md")
    print()
    print("Happy robotics! 🤖✨")
    print("="*60)

if __name__ == "__main__":
    main()
