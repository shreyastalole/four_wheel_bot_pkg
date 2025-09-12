#!/bin/bash

# Simple launcher script for the four wheel bot
echo "============================================"
echo "    Four Wheel Bot Launcher"
echo "============================================"
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 not found. Please source your ROS2 setup first:"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "source install/setup.bash"
    exit 1
fi

# Build the package first
echo "Building four_wheel_bot_pkg..."
colcon build --packages-select four_wheel_bot_pkg
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Source the workspace
source install/setup.bash

# Ask user which robot type
echo ""
echo "Which robot would you like to launch?"
echo "1) Differential Drive Bot (tank-style)"
echo "2) Ackermann Steering Bot (car-style)"
read -p "Enter choice (1 or 2): " choice

case $choice in
    1)
        robot_type="diff_drive"
        echo "Launching Differential Drive Bot..."
        ;;
    2)
        robot_type="ackermann"
        echo "Launching Ackermann Steering Bot..."
        ;;
    *)
        echo "Invalid choice. Defaulting to Differential Drive Bot..."
        robot_type="diff_drive"
        ;;
esac

echo ""
echo "Starting Gazebo and robot..."
echo "A new terminal will open for teleop control."
echo ""

# Launch the robot in Gazebo
ros2 launch four_wheel_bot_pkg ${robot_type}_bot.launch.py &
robot_pid=$!

# Wait a bit for Gazebo to start
sleep 5

# Open teleop in new terminal
gnome-terminal -- bash -c "
    echo 'Four Wheel Bot Teleop Control';
    echo '============================';
    echo 'Commands:';
    echo '  w = forward';
    echo '  s = backward';  
    echo '  a = left';
    echo '  d = right';
    echo '  x = stop';
    echo '  q = quit';
    echo '';
    source install/setup.bash;
    ros2 run four_wheel_bot_pkg simple_teleop.py;
    exec bash
" &

echo "Robot launched! Use the teleop terminal to control the robot."
echo "Press Ctrl+C here to stop everything."

# Wait for user to stop
wait $robot_pid
