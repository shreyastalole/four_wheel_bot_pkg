# Four Wheel Bot Package

This package provides two different 4-wheel robot configurations for ROS2 Jazzy and Gazebo Harmonic:

1. **Differential Drive Bot** - Tank-style movement where left and right wheels are controlled independently
2. **Ackermann Steering Bot** - Car-style movement with front wheel steering and rear wheel drive

## Features

- Compatible with ROS2 Jazzy and Gazebo Harmonic
- WASD keyboard control like a game
- Two distinct robot models with different control mechanisms
- Simple world environment for testing
- Clean, readable code following ROS2 best practices

## Package Structure

```
four_wheel_bot_pkg/
├── urdf/                           # Robot descriptions
│   ├── diff_drive_bot.urdf.xacro   # Differential drive robot
│   ├── ackermann_bot.urdf.xacro    # Ackermann steering robot
│   ├── materials.xacro             # Material definitions
│   └── properties.xacro            # Robot properties/parameters
├── launch/                         # Launch files
│   ├── diff_drive_bot.launch.py    # Launch differential drive robot
│   ├── ackermann_bot.launch.py     # Launch ackermann robot
│   └── teleop_bot.launch.py        # Launch robot with teleop
├── worlds/                         # Gazebo world files
│   └── simple_world.sdf            # Simple test world
├── scripts/                        # Python scripts
│   └── keyboard_teleop.py          # WASD keyboard controller
└── config/                         # Configuration files
```

## Usage

### Building the Package

```bash
cd ~/ros2_ws
colcon build --packages-select four_wheel_bot_pkg
source install/setup.bash
```

### Quick Start (Recommended)

```bash
# Launch differential drive robot with simple teleop (default)
ros2 launch four_wheel_bot_pkg teleop_bot.launch.py

# Launch ackermann robot with simple teleop
ros2 launch four_wheel_bot_pkg teleop_bot.launch.py robot_type:=ackermann
```

### Manual Launch (Two Terminals)

```bash
# Terminal 1: Launch robot in Gazebo
ros2 launch four_wheel_bot_pkg diff_drive_bot.launch.py
# or
ros2 launch four_wheel_bot_pkg ackermann_bot.launch.py

# Terminal 2: Launch teleop control
ros2 run four_wheel_bot_pkg simple_teleop.py
# or (if you prefer keyboard mode)
ros2 run four_wheel_bot_pkg keyboard_teleop.py
```

## Controls

### Simple Teleop (Recommended)
Enter commands in the terminal:
- **w**: Move forward
- **s**: Move backward  
- **a**: Turn left / Steer left
- **d**: Turn right / Steer right
- **x**: Stop immediately
- **q**: Quit teleop

### Keyboard Teleop (Alternative)
Real-time WASD controls (press and hold):
- **W/w**: Move forward
- **S/s**: Move backward  
- **A/a**: Turn left / Steer left
- **D/d**: Turn right / Steer right
- **Q/q**: Increase speed
- **E/e**: Decrease speed
- **Space**: Stop immediately
- **Ctrl+C**: Exit teleop

## Robot Specifications

### Differential Drive Bot (Blue)
- **Control Method**: Tank-style differential drive
- **Movement**: Independent left/right wheel control
- **Best For**: Precise maneuvering, spot turns, obstacle navigation
- **Wheels**: All 4 wheels are driven

### Ackermann Steering Bot (Red)
- **Control Method**: Car-like ackermann steering
- **Movement**: Front wheels steer, rear wheels drive
- **Best For**: Smooth turns, highway-style navigation
- **Wheels**: Front 2 wheels steer, rear 2 wheels drive

## Technical Details

- **ROS2 Version**: Jazzy
- **Gazebo Version**: Harmonic
- **Physics**: Gazebo physics with realistic inertias
- **Plugins**: gz-sim system plugins for modern Gazebo
- **Topics**: Standard `/cmd_vel` for control, `/odom` for odometry

## Troubleshooting

1. **Gazebo doesn't start**: Make sure Gazebo Harmonic is installed
2. **Robot doesn't move**: Check that the bridge node is running
3. **URDF errors**: Verify xacro is installed and working
4. **Keyboard doesn't work**: Make sure the teleop terminal has focus

## Dependencies

- ROS2 Jazzy
- Gazebo Harmonic
- ros_gz_sim
- ros_gz_bridge
- robot_state_publisher
- xacro

## Future Enhancements

- Add sensor integration (lidar, camera)
- Implement autonomous navigation
- Add more complex world environments
- Create parameter files for easy customization
