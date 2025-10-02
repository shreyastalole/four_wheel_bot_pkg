# Four Wheel Bot Package

A comprehensive ROS2 package providing an Ackermann steering 4-wheel robot with camera sensor, LiDAR, and multiple simulation environments for autonomous driving research and education.

## 🚗 Features

- **Ackermann Steering Robot** - Car-like movement with front wheel steering and rear wheel drive
- **Multi-Sensor Suite** - RGB camera (640x480) and LiDAR sensor for perception
- **Multiple Worlds** - Simple test world, road track, and Sonoma raceway
- **ROS2 Jazzy** and **Gazebo Harmonic** compatibility
- **Teleoperation Support** - Standard teleop_twist_keyboard interface
- **Sensor Visualization Tools** - Camera viewer, LiDAR test scripts
- **Scalable Robot Model** - Can be scaled to different sizes (e.g., Prius-sized)
- **Clean Architecture** - Following ROS2 best practices with proper physics simulation

## 📁 Package Structure

```
four_wheel_bot_pkg/
├── urdf/                           # Robot descriptions
│   ├── ackermann_bot.urdf.xacro    # Main Ackermann robot model
│   ├── materials.xacro             # Material definitions  
│   └── properties.xacro            # Robot parameters
├── launch/                         # Launch files
│   ├── robot_launch.py             # Basic robot launch
│   ├── teleop_launch.py            # Robot + teleop (simple world)
│   ├── robot_with_lidar_test.launch.py # Robot with LiDAR testing
│   ├── rtn_teleop_launch.py        # Road track navigation
│   └── sonoma_teleop_launch.py     # Sonoma raceway
├── worlds/                         # Gazebo environments
│   ├── simple_world.sdf            # Basic test environment
│   ├── road_track_normalised_new.sdf # Custom road track
│   └── sonoma.sdf                  # Sonoma raceway track
├── models/                         # 3D models and meshes
│   └── *.obj                       # Track and environment meshes
├── scripts/                        # Control and testing scripts
│   ├── teleop_twist_keyboard.py    # Standard ROS2 teleop
│   ├── view_camera.py              # Camera viewer
│   ├── lidar_test.py               # Basic LiDAR testing
│   ├── lidar_test_3d.py            # Advanced LiDAR analysis
│   ├── check_lidar_topics.py       # Topic diagnostics
│   └── demo_info.py                # Package information
└── config/                         # Configuration files
    └── camera_view.rviz            # RViz camera visualization
```

## 📋 Dependencies & Installation

### System Requirements
- Ubuntu 22.04 LTS
- ROS2 Jazzy Jalopy
- Gazebo Harmonic

### Install Dependencies
```bash
# Core ROS2 packages
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-xacro \
                 ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-cv-bridge \
                 ros-jazzy-image-view \
                 ros-jazzy-rviz2

# Gazebo Harmonic
sudo apt install gz-harmonic

# Python dependencies
sudo apt install python3-opencv-dev

# Terminal emulator for teleop (REQUIRED)
sudo apt install xterm

# Optional: Alternative terminal emulators
sudo apt install gnome-terminal konsole
```

## 🚀 Quick Start

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select four_wheel_bot_pkg
source install/setup.bash
```

### 2. Launch Options

#### Basic Robot (Simple World)
```bash
# Robot + Teleop together
ros2 launch four_wheel_bot_pkg teleop_launch.py

# Or manual control (two terminals)
# Terminal 1: Launch robot
ros2 launch four_wheel_bot_pkg robot_launch.py
# Terminal 2: Run teleop
ros2 run four_wheel_bot_pkg teleop_twist_keyboard.py
```

#### Road Track Navigation
```bash
# Launch robot on custom road track
ros2 launch four_wheel_bot_pkg rtn_teleop_launch.py
```

#### Sonoma Raceway
```bash
# Launch robot on Sonoma raceway
ros2 launch four_wheel_bot_pkg sonoma_teleop_launch.py
```

#### With LiDAR Testing
```bash
# Launch robot with LiDAR sensor testing
ros2 launch four_wheel_bot_pkg robot_with_lidar_test.launch.py
```

### 3. View Sensor Data

#### Camera Feed
```bash
# Method 1: Python OpenCV viewer (recommended)
ros2 run four_wheel_bot_pkg view_camera.py

# Method 2: Standard image viewer
ros2 run image_view image_view --ros-args --remap image:=/world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image

# Method 3: RViz2 visualization
ros2 run rviz2 rviz2 -d src/four_wheel_bot_pkg/config/camera_view.rviz
```

#### LiDAR Data
```bash
# Basic LiDAR test
ros2 run four_wheel_bot_pkg lidar_test.py

# Advanced 3D LiDAR analysis
ros2 run four_wheel_bot_pkg lidar_test_3d.py

# Check LiDAR topics
ros2 run four_wheel_bot_pkg check_lidar_topics.py
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
- **Scaling**: Configurable size (default can be scaled to Prius dimensions)

### Sensor Suite
#### Camera Sensor
- **Resolution**: 640x480 pixels
- **Format**: RGB (R8G8B8)
- **Field of View**: 60 degrees (1.0472 radians)
- **Update Rate**: 10 Hz
- **Position**: Front-mounted, 50mm above chassis
- **Topic**: `/world/{world_name}/model/ackermann_bot/link/base_link/sensor/camera_sensor/image`
- **Message Type**: `sensor_msgs/msg/Image`

#### LiDAR Sensor
- **Type**: 2D Laser Scanner
- **Range**: 0.1m to 10m
- **Resolution**: 360 points per scan
- **Update Rate**: 10 Hz
- **Topics**: 
  - LaserScan: `/world/{world_name}/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan`
  - PointCloud2: `/world/{world_name}/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan/points`

## 🗺️ Available Worlds

1. **Simple World (`simple_world.sdf`)**
   - Basic flat ground plane
   - Good for initial testing and development

2. **Road Track (`road_track_normalised_new.sdf`)**
   - Custom road track with curves
   - Realistic driving environment
   - Uses 3D mesh models

3. **Sonoma Raceway (`sonoma.sdf`)**
   - Professional racing circuit
   - Complex track layout
   - Challenging navigation environment

## 🔧 Technical Details

- **ROS2 Version**: Jazzy Jalopy
- **Gazebo Version**: Harmonic
- **Control Topic**: `/cmd_vel` (geometry_msgs/Twist)
- **Camera Topics**: `/world/{world_name}/model/ackermann_bot/link/base_link/sensor/camera_sensor/image`
- **LiDAR Topics**: `/world/{world_name}/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan`
- **Bridge**: ros_gz_bridge for ROS-Gazebo communication
- **Physics**: gz-sim with ackermann steering system and realistic sensors

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

### LiDAR not working
1. Check available LiDAR topics:
   ```bash
   ros2 topic list | grep lidar
   ```
2. Test LiDAR data:
   ```bash
   ros2 topic echo /world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan --once
   ```

### Teleop doesn't respond
- Make sure the teleop terminal (xterm) has focus
- Verify xterm is installed: `sudo apt install xterm`
- Try manual method with separate terminals
- Check that no other processes are publishing to `/cmd_vel`

### World loading errors
- Ensure mesh files are present in the `models/` directory
- Check that world files reference correct mesh paths
- Verify package is properly built and sourced

### Gazebo doesn't start
- Ensure Gazebo Harmonic is properly installed
- Check that `gz sim` command works independently
- Try launching with verbose output: `gz sim -v 4`

## 📊 Testing & Diagnostics

```bash
# Check all topics
ros2 topic list

# Monitor sensor data rates
ros2 topic hz /world/simple_world/model/ackermann_bot/link/base_link/sensor/camera_sensor/image
ros2 topic hz /world/simple_world/model/ackermann_bot/link/base_link/sensor/lidar_sensor/scan

# View robot model in RViz
ros2 launch four_wheel_bot_pkg robot_launch.py
ros2 run rviz2 rviz2

# Package information
ros2 run four_wheel_bot_pkg demo_info.py
```

## 🔮 Future Enhancements

- [x] **Multi-sensor integration** - Camera + LiDAR ✅
- [x] **Multiple simulation worlds** - Various environments ✅
- [x] **Sensor testing utilities** - Diagnostic scripts ✅
- [ ] Add GPS and IMU sensors
- [ ] Implement SLAM (Simultaneous Localization and Mapping)
- [ ] Add autonomous navigation stack
- [ ] Create path planning demonstrations
- [ ] Integrate machine learning for perception tasks
- [ ] Add weather and lighting variations to worlds

## 📝 Notes

- Robot spawns at coordinates (0, 0, 0.2) to prevent floor collision
- All launch files include proper timing delays for reliable startup
- Camera and LiDAR sensors work in VirtualBox despite graphics warnings
- World names in topics change based on the selected world file
- Bridge automatically handles Gazebo↔ROS2 message conversion
- xterm is required for teleop keyboard functionality

---

**Version**: 3.0 (Multi-Sensor & Multi-World)  
**Tested on**: Ubuntu 22.04, ROS2 Jazzy, Gazebo Harmonic, VirtualBox  
**License**: Apache 2.0