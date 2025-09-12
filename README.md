# Four Wheel Bot Package

A clean, well-structured ROS2 package providing an Ackermann steering 4-wheel robot for simulation and teleoperation.

## 🚗 Features

- **Ackermann Steering Robot** - Car-style movement with front wheel steering and rear wheel drive
- **ROS2 Jazzy** and **Gazebo Harmonic** compatibility
- **Standard teleop_twist_keyboard** interface for reliable control
- **Clean, minimal codebase** following ROS2 best practices
- **Proper physics simulation** with realistic inertias and collisions

## 📁 Package Structure

```
four_wheel_bot_pkg/
├── urdf/                           # Robot descriptions
│   ├── ackermann_bot.urdf.xacro    # Main Ackermann robot model
│   ├── materials.xacro             # Material definitions  
│   └── properties.xacro            # Robot parameters
├── launch/                         # Launch files
│   ├── robot_launch.py             # Launch robot only
│   └── teleop_launch.py            # Launch robot + teleop
├── worlds/                         # Gazebo environments
│   └── simple_world.sdf            # Simple test world
└── scripts/                        # Control scripts
    └── teleop_twist_keyboard.py    # Standard ROS2 teleop
```

## 🚀 Quick Start

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select four_wheel_bot_pkg
source install/setup.bash
```

### 2. Launch Robot (Recommended Method)
```bash
# Method 1: Robot + Teleop together
ros2 launch four_wheel_bot_pkg teleop_launch.py

# Method 2: Manual control (most reliable)
# Terminal 1: Launch robot
ros2 launch four_wheel_bot_pkg robot_launch.py

# Terminal 2: Run teleop
ros2 run four_wheel_bot_pkg teleop_twist_keyboard.py
```

## 🎮 Controls

The robot uses standard `teleop_twist_keyboard` controls:

### Movement Keys
- **`i`** - Move forward
- **`,`** - Move backward  
- **`j`** - Turn left
- **`l`** - Turn right
- **`k`** - Stop
- **`u`** - Forward + left
- **`o`** - Forward + right
- **`m`** - Backward + left
- **`.`** - Backward + right

### Speed Control
- **`q/z`** - Increase/decrease all speeds by 10%
- **`w/x`** - Increase/decrease only linear speed by 10%
- **`e/c`** - Increase/decrease only angular speed by 10%

### Exit
- **`Ctrl+C`** - Quit teleop

## 🤖 Robot Specifications

### Ackermann Steering Bot
- **Color**: Red chassis, dark grey wheels
- **Control**: Car-like ackermann steering
- **Steering**: Front wheels steer (±45°)
- **Drive**: Rear wheels provide propulsion
- **Physics**: Proper collision detection and inertial properties
- **Spawn Height**: 20cm above ground (prevents floor clipping)

## 🔧 Technical Details

- **ROS2 Version**: Jazzy Jalopy
- **Gazebo Version**: Harmonic
- **Control Topic**: `/cmd_vel` (geometry_msgs/Twist)
- **Odometry Topic**: `/odom` 
- **Bridge**: ros_gz_bridge for ROS-Gazebo communication
- **Physics**: gz-sim with ackermann steering system

## 📋 Dependencies

```bash
# Core ROS2 packages
ros-jazzy-robot-state-publisher
ros-jazzy-xacro

# Gazebo integration
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-bridge

# Gazebo Harmonic
gz-harmonic
```

## 🛠 Troubleshooting

### Robot doesn't move
1. Check if `/cmd_vel` commands are being published:
   ```bash
   ros2 topic echo /cmd_vel
   ```
2. Verify bridge is running:
   ```bash
   ros2 node list | grep parameter_bridge
   ```

### Robot is "swimming" in floor
- This has been fixed with proper spawn height (z=0.2)
- Robot chassis is positioned correctly above ground

### Teleop doesn't respond
- Make sure the teleop terminal has focus
- Try the manual method (separate terminals)
- Check that no other processes are publishing to `/cmd_vel`

### Gazebo doesn't start
- Ensure Gazebo Harmonic is properly installed
- Check that `gz sim` command works independently

## 🔮 Future Enhancements

- [ ] Add differential drive variant
- [ ] Integrate sensors (lidar, camera, IMU)
- [ ] Implement autonomous navigation
- [ ] Add more complex world environments
- [ ] Create launch parameters for easy customization
- [ ] Add rviz2 visualization

## 📝 Notes

- Robot spawns at coordinates (0, 0, 0.2) to prevent floor collision
- Wheel positions are carefully calibrated for proper ground contact
- Uses standard ROS2 conventions for maximum compatibility

---

**Version**: 1.0  
**Tested on**: Ubuntu 22.04, ROS2 Jazzy, Gazebo Harmonic
