# Four Wheel Bot Package

A clean, well-structured ROS2 package providing an Ackermann steering 4-wheel robot with camera sensor for simulation and teleoperation.

## 🚗 Features

- **Ackermann Steering Robot** - Car-style movement with front wheel steering and rear wheel drive
- **Front-mounted Camera Sensor** - 640x480 RGB camera for computer vision applications
- **ROS2 Jazzy** and **Gazebo Harmonic** compatibility
- **Standard teleop_twist_keyboard** interface for reliable control
- **Camera Visualization Tools** - Multiple ways to view camera feed
- **Clean, minimal codebase** following ROS2 best practices
- **Proper physics simulation** with realistic inertias and collisions

## 📁 Package Structure

```
four_wheel_bot_pkg/
├── urdf/                           # Robot descriptions
│   ├── ackermann_bot.urdf.xacro    # Main Ackermann robot model with camera
│   ├── materials.xacro             # Material definitions  
│   └── properties.xacro            # Robot parameters
├── launch/                         # Launch files
│   ├── robot_launch.py             # Launch robot only
│   └── teleop_launch.py            # Launch robot + teleop + camera
├── worlds/                         # Gazebo environments
│   ├── simple_world.sdf            # Simple test world
│   ├── road_grid_world.sdf         # Road grid environment
│   └── sonoma.sdf                  # Sonoma raceway track
├── scripts/                        # Control and visualization scripts
│   ├── teleop_twist_keyboard.py    # Standard ROS2 teleop
│   └── view_camera.py              # Camera viewer script
└── config/                         # Configuration files
    └── camera_view.rviz            # RViz camera visualization config
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
# Method 1: Robot + Teleop together (includes camera)
ros2 launch four_wheel_bot_pkg teleop_launch.py

# Method 2: Manual control (most reliable)
# Terminal 1: Launch robot
ros2 launch four_wheel_bot_pkg robot_launch.py

# Terminal 2: Run teleop
ros2 run four_wheel_bot_pkg teleop_twist_keyboard.py
```

### 3. View Camera Feed
```bash
# Method 1: Python OpenCV viewer (recommended)
python3 src/four_wheel_bot_pkg/scripts/view_camera.py

# Method 2: RViz2 visualization
ros2 run rviz2 rviz2 -d src/four_wheel_bot_pkg/config/camera_view.rviz

# Method 3: Standard image viewer
ros2 run image_view image_view --ros-args --remap image:=/world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image
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
- **Color**: Red chassis, blue camera, dark grey wheels
- **Control**: Car-like ackermann steering
- **Steering**: Front wheels steer (±45°)
- **Drive**: Rear wheels provide propulsion
- **Physics**: Proper collision detection and inertial properties
- **Spawn Height**: 20cm above ground (prevents floor clipping)

### Camera Sensor
- **Resolution**: 640x480 pixels
- **Format**: RGB (R8G8B8)
- **Field of View**: 60 degrees (1.0472 radians)
- **Update Rate**: 10 Hz (limited by VirtualBox performance)
- **Position**: Front-mounted, 50mm above chassis
- **Topic**: `/world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image`
- **Message Type**: `sensor_msgs/msg/Image`

## 🔧 Technical Details

- **ROS2 Version**: Jazzy Jalopy
- **Gazebo Version**: Harmonic
- **Control Topic**: `/cmd_vel` (geometry_msgs/Twist)
- **Odometry Topic**: `/odom` 
- **Camera Topic**: `/world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image`
- **Bridge**: ros_gz_bridge for ROS-Gazebo communication
- **Physics**: gz-sim with ackermann steering system and camera sensors

## 📋 Dependencies

```bash
# Core ROS2 packages
ros-jazzy-robot-state-publisher
ros-jazzy-xacro

# Gazebo integration
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-bridge

# Camera and visualization
ros-jazzy-cv-bridge
ros-jazzy-image-view
ros-jazzy-rviz2

# Gazebo Harmonic
gz-harmonic

# Python dependencies
python3-opencv-dev
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

### Camera not working
1. Check if camera topic is available:
   ```bash
   ros2 topic list | grep camera
   ```
2. Verify camera is publishing:
   ```bash
   ros2 topic hz /world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image
   ```
3. Check for graphics warnings (normal in VirtualBox):
   - VMware/libEGL warnings are expected and don't prevent camera from working

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

- [x] **Add camera sensor** - Front-mounted RGB camera ✅
- [x] **Camera visualization tools** - Multiple viewing methods ✅
- [ ] Add more sensors (lidar, IMU, depth camera)
- [ ] Implement autonomous navigation with camera input
- [ ] Add object detection and tracking
- [ ] Create launch parameters for easy customization
- [ ] Add differential drive variant
- [ ] Integrate machine learning for vision tasks

## 📸 Camera Features

### Current Implementation
- **RGB Camera**: 640x480 resolution with 60° FOV
- **Real-time Streaming**: Camera data available as ROS2 topic
- **Multiple Viewers**: OpenCV, RViz2, and image_view support
- **Proper Integration**: Works with both robot and teleop launch files

### Usage Examples
```bash
# Start robot with camera
ros2 launch four_wheel_bot_pkg teleop_launch.py

# View camera feed
python3 src/four_wheel_bot_pkg/scripts/view_camera.py

# Drive around and see camera perspective
# Use 'i','j','l',',','k' keys to move robot
```

## 📝 Notes

- Robot spawns at coordinates (0, 0, 0.2) to prevent floor collision
- Wheel positions are carefully calibrated for proper ground contact
- Camera is mounted 50mm above chassis at the front for optimal view
- Camera sensor works in VirtualBox despite graphics warnings
- Uses standard ROS2 conventions for maximum compatibility
- Bridge automatically handles Gazebo↔ROS2 message conversion

---

**Version**: 2.0 (Camera Integration)  
**Branch**: dev-sensors  
**Tested on**: Ubuntu 22.04, ROS2 Jazzy, Gazebo Harmonic, VirtualBox
