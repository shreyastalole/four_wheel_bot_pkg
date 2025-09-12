#!/bin/bash

echo "============================================"
echo "    Four Wheel Bot Test Script"
echo "============================================"
echo ""

# Source workspace
cd /home/jetracer/ros2_ws
source install/setup.bash

echo "Step 1: Testing URDF processing..."
xacro src/four_wheel_bot_pkg/urdf/diff_drive_bot.urdf.xacro > /tmp/test_robot.urdf
if [ $? -eq 0 ]; then
    echo "✓ URDF processing works"
else
    echo "✗ URDF processing failed"
    exit 1
fi

echo ""
echo "Step 2: Testing package installation..."
if ros2 pkg list | grep -q four_wheel_bot_pkg; then
    echo "✓ Package is installed"
else
    echo "✗ Package not found"
    exit 1
fi

echo ""
echo "Step 3: Testing executables..."
if [ -f "install/four_wheel_bot_pkg/lib/four_wheel_bot_pkg/simple_teleop.py" ]; then
    echo "✓ Teleop script is installed"
else
    echo "✗ Teleop script not found"
    exit 1
fi

echo ""
echo "Step 4: Testing bridge availability..."
if ros2 pkg list | grep -q ros_gz_bridge; then
    echo "✓ Gazebo bridge is available"
else
    echo "✗ Gazebo bridge not found"
    exit 1
fi

echo ""
echo "Step 5: Starting manual test..."
echo "This will launch the robot step by step"
echo ""

# Launch Gazebo with empty world first
echo "Starting Gazebo..."
gz sim -v 4 src/four_wheel_bot_pkg/worlds/simple_world.sdf &
gazebo_pid=$!
sleep 3

# Start robot state publisher
echo "Starting robot state publisher..."
xacro src/four_wheel_bot_pkg/urdf/diff_drive_bot.urdf.xacro | ros2 param set robot_state_publisher robot_description -
ros2 run robot_state_publisher robot_state_publisher &
rsp_pid=$!
sleep 2

# Spawn robot
echo "Spawning robot..."
ros2 run ros_gz_sim create -topic robot_description -name test_diff_bot -x 0 -y 0 -z 0.1 &
spawn_pid=$!
sleep 2

# Start bridge
echo "Starting bridge..."
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
bridge_pid=$!
sleep 2

echo ""
echo "Robot should now be in Gazebo!"
echo "Test by running in another terminal:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}' --once"
echo ""
echo "Press Ctrl+C to stop all processes"

# Wait for interrupt
trap "kill $gazebo_pid $rsp_pid $spawn_pid $bridge_pid 2>/dev/null; exit" INT
wait $gazebo_pid
