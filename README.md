# Differential Drive Robot Simulation

A comprehensive ROS 2 package for simulating a differential drive robot in Gazebo Fortress with integrated sensors, standard ROS 2 interfaces, and visualization support.

## Features

✅ **Gazebo Fortress Integration**
- Full Gazebo Fortress (Ignition) simulation support
- Modern physics engine with configurable parameters
- Native ROS 2 topic bridging

✅ **Complete Robot Model**
- Differential drive kinematics
- IMU sensor with realistic noise simulation
- 2D LiDAR with 360° field of view (3.5m range)
- Front caster wheel for stability
- Comprehensive collision models

✅ **Gazebo Sensors & Actuators**
- Odometry publishing from simulated wheel encoders
- IMU sensor data (acceleration, angular velocity)
- LiDAR scan data for navigation
- Differential drive actuators

✅ **Easy Integration**
- Standard `/cmd_vel` interface for control commands
- Works with any ROS 2 teleop tool (native or TurtleBot 3)
- Compatible with Nav2 navigation stack
- RViz2 visualization ready

✅ **Nav2 Navigation Stack**
- Full AMCL localization (particle filter)
- Dynamic path planning with obstacle avoidance
- Behavior tree-based navigation
- Pre-configured parameters for differential drive robots
- Integrated map handling with occupancy grid
- Recovery behaviors for stuck situations

✅ **Visualization**
- RViz2 integration with pre-configured display
- Real-time TF tree visualization
- Sensor data visualization
- Odometry tracking

## Package Structure

```
diff_drive_sim/
├── urdf/
│   ├── diff_drive_robot.urdf        # Main URDF robot description
│   ├── diff_drive_robot.xacro       # Xacro macros and parameters
│   └── diff_drive_robot.sdf         # Generated SDF file
├── config/
│   ├── bridge_config.yaml           # ROS-Gazebo topic bridge configuration
│   └── nav_params.yaml              # Nav2 parameters (AMCL, planners, controllers)
├── launch/
│   ├── sim.launch.py                # Main simulation launcher
│   └── nav.launch.py                # Navigation stack launcher (localization + navigation)
├── maps/
│   ├── map.yaml                     # Map metadata (origin, resolution, image)
│   └── map.pgm                      # Map occupancy grid image
├── world/
│   ├── default.sdf                  # Default simulation world
│   └── my_world.sdf                 # Alternative world
├── rviz/
│   └── rviz.config                  # RViz visualization configuration
├── CMakeLists.txt
├── package.xml
├── README.md                        # This file
└── CHANGE_LOG.md                    # Version history
```

## Quick Start

### Prerequisites

```bash
# Install ROS 2 Humble
sudo apt-get install ros-humble-desktop

# Install core simulation packages
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-gazebo-plugins \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-rviz2 \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-teleop-twist-joy \
                 ros-humble-joy

# Install Gazebo Fortress
sudo apt install ignition-fortress ros-humble-ros-gz-sim ros-humble-ros-gz-bridge

# Install Nav2 stack (for autonomous navigation)
sudo apt install ros-humble-nav2-* \
                 ros-humble-slam-toolbox
```

### Build the Package

```bash
cd ~/work/agv_ws
colcon build --packages-select diff_drive_sim
source install/setup.bash
```

## Usage Guide

### 1. Start Simulation

**Terminal 1: Launch simulation with GUI**
```bash
ros2 launch diff_drive_sim sim.launch.py
```

**Alternative: Launch simulation headless (no GUI)**
```bash
ros2 launch diff_drive_sim sim.launch.py headless:=true
```

**Launch options:**
- `use_sim_time:=true/false` - Use Gazebo simulation time
- `headless:=true/false` - Disable/enable Gazebo GUI
- `verbose:=true/false` - Enable/disable verbose output

### 2. Teleoperation Control

This package provides the simulation only. For teleoperation, use native ROS 2 tools:

#### **Option A: Native ROS 2 Keyboard Teleop**

```bash
# Terminal 2: Start keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard Controls:**
```
     i
   j k l       - Move forward/left/backward/right
     
   u o         - Rotate counter-clockwise/clockwise
   
   m ,         - Increase/decrease speed
   space       - Stop
```

---

#### **Option B: TurtleBot 3 Keyboard Teleop**

For TurtleBot 3-compatible control, install turtlebot3_teleop:

```bash
sudo apt install ros-humble-turtlebot3-teleop
```

Then run:

```bash
# Terminal 2: Start TurtleBot 3 style keyboard teleop
ros2 run turtlebot3_teleop teleop_keyboard
```

**TurtleBot 3 Keyboard Controls:**
```
w/x       - Increase/Decrease Linear Velocity
a/d       - Increase/Decrease Angular Velocity
space     - Force Stop
q/z       - Reset Linear/Angular Velocity
```

---

#### **Option C: Joystick/Gamepad Control**

```bash
# Terminal 2: Install and run joy
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy

# Terminal 2: Start joy node
ros2 run joy joy_node

# Terminal 3: Start teleop with joystick
ros2 run teleop_twist_joy teleop_twist_joy
```

**Joystick Mapping (default):**
- **Left Stick (Y-axis)** - Linear velocity
- **Right Stick (X-axis)** - Angular velocity
- **Button 0** - Enable/Disable
- **Button 5** - Turbo mode (2x speed)

---

#### **Option D: Direct ROS 2 Topic Publishing**

```bash
# Publish velocity command directly to /cmd_vel
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"

# Example: Forward with rotation
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"

# Example: Backward
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.2}}"
```

### 3. Monitor Robot State

**View available topics:**
```bash
ros2 topic list
```

**Echo specific topics:**
```bash
# Odometry
ros2 topic echo /robot/odom

# IMU data
ros2 topic echo /robot/imu

# Laser scan
ros2 topic echo /scan

# Velocity commands
ros2 topic echo /cmd_vel

# Joint states
ros2 topic echo /joint_states
```

**View TF tree:**
```bash
# In RViz or terminal
ros2 run tf2_tools view_frames.py
```

**Check TF transforms:**
```bash
ros2 run tf2_ros tf2_echo base_link drivewhl_l_link
```

### 4. Navigation Stack (Autonomous Navigation)

The package includes full Nav2 integration with pre-configured parameters optimized for differential drive robots.

#### **Start Navigation**

```bash
# Terminal 1: Start simulation
ros2 launch diff_drive_sim sim.launch.py

# Terminal 2: Start navigation stack (AMCL + Path Planning)
ros2 launch diff_drive_sim nav.launch.py
```

#### **Navigation Launch Options**

```bash
# Custom map file
ros2 launch diff_drive_sim nav.launch.py map:=/path/to/your/map.yaml

# Custom navigation parameters
ros2 launch diff_drive_sim nav.launch.py nav_params_file:=/path/to/nav_params.yaml

# Disable autostart (manual server startup)
ros2 launch diff_drive_sim nav.launch.py autostart:=false
```

#### **Using Navigation in RViz**

1. Open RViz2 and load `rviz.config` or create a new config
2. Add these displays:
   - **Map** - Shows the occupancy grid
   - **Pose Estimate** - Use "2D Pose Estimate" tool to set initial pose
   - **Path** - Shows planned path from planner
   - **Particle Cloud** - Shows AMCL particle filter
   - **LaserScan** - Shows robot's LiDAR data
3. Use **2D Goal Pose** tool to set navigation goals
4. Robot will autonomously navigate while avoiding obstacles

#### **Navigation Parameters**

Key AMCL parameters in `config/nav_params.yaml`:
- `max_particles: 2000` - Number of particles in filter
- `laser_model_type: "likelihood_field"` - Laser matching model
- `robot_model_type: "DifferentialMotionModel"` - Motion model for diff drive
- `update_min_d: 0.25` - Minimum linear distance between updates (m)
- `update_min_a: 0.2` - Minimum angular distance between updates (rad)
- `laser_likelihood_max_dist: 2.0` - Max distance for beam correspondence (m)

#### **Manual Robot Positioning**

If robot gets lost:

```bash
# In RViz2: Use "2D Pose Estimate" tool
# Or via CLI:
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, 
    pose: {pose: {position: {x: 0.0, y: 0.0}, 
    orientation: {w: 1.0}}, 
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891458577149]}}"
```

#### **Map Configuration**

Map files are located in `maps/`:
- `map.yaml` - Map metadata (YAML format)
- `map.pgm` - Occupancy grid image (grayscale PNG/PGM)

Map metadata format:
```yaml
image: map.pgm           # Path to occupancy grid image
resolution: 0.05         # Resolution in m/pixel (how many meters each pixel represents)
origin: [0.0, 0.0, 0.0]  # Origin position (x, y, theta)
negate: 0                # Negate colors (0 or 1)
occupied_thresh: 0.65    # Occupancy threshold (0-1)
free_thresh: 0.25        # Free space threshold (0-1)
```

## Robot Specifications

### Physical Dimensions
| Component | Value |
|-----------|-------|
| **Length** | 0.42 m |
| **Width** | 0.31 m |
| **Height** | 0.18 m |
| **Mass** | ~3.5 kg |
| **Wheel Radius** | 0.1 m |
| **Wheel Separation** | 0.4 m |
| **Caster Radius** | 0.06 m |

### Sensors
| Sensor | Specifications |
|--------|----------------|
| **IMU** | 100 Hz, Gaussian noise |
| **LiDAR** | 360°, 3.5m range, 5 Hz |
| **Odometry** | Calculated from wheel encoders |

### Performance
| Parameter | Value |
|-----------|-------|
| **Max Linear Speed** | 1.0 m/s |
| **Max Angular Speed** | 2.0 rad/s |
| **Update Rate** | 50 Hz |
| **Simulation Time Step** | 0.001 s (1 ms) |

## ROS 2 Topics

### Published Topics
```
/robot/odom                     (nav_msgs/Odometry)       - Robot odometry
/robot/imu                      (sensor_msgs/Imu)         - IMU data
/scan                           (sensor_msgs/LaserScan)   - LiDAR data
/joint_states                   (sensor_msgs/JointState)  - Joint positions/velocities
/tf                             (tf2_msgs/TFMessage)      - Transform frames
/clock                          (rosgraph_msgs/Clock)     - Simulation clock
```

### Subscribed Topics
```
/cmd_vel                        (geometry_msgs/Twist)     - Velocity commands
/controller_manager/*           (controller_manager msgs) - Controller commands
```

## Launch Configuration

### sim.launch.py
Main simulation launcher that starts:
- Gazebo server (physics simulation)
- Gazebo client (GUI, optional via `headless` flag)
- Robot spawner
- Robot state publisher (with xacro processing)
- Joint state publisher for TF tree
- ROS-Gazebo bridge for topic bridging
- RViz2 (optional)

**Available arguments:**
- `use_sim_time:=true/false` - Use Gazebo simulation time
- `headless:=true/false` - Disable/enable Gazebo GUI
- `verbose:=true/false` - Enable verbose logging

### nav.launch.py
Navigation stack launcher that starts:
- **Localization**: AMCL (Adaptive Monte Carlo Localization) for robot pose estimation
- **Navigation**: Nav2 with path planning, costmap, and behavior tree
- Pre-loads map and navigation parameters
- Integrates with Gazebo simulation via sim_time

**Available arguments:**
- `map:=/path/to/map.yaml` - Custom map file (default: package maps/map.yaml)
- `nav_params_file:=/path/to/nav_params.yaml` - Custom nav parameters
- `use_sim_time:=true/false` - Use simulation time
- `autostart:=true/false` - Automatically start navigation servers

## Configuration Files

### bridge_config.yaml
Defines ROS 2 ↔ Gazebo topic mappings:
```yaml
# Topic bridge configuration for ros_gz_bridge
# Maps ROS 2 topics to Gazebo topics
```

## Troubleshooting

### Robot not responding to /cmd_vel
```bash
# Check if bridge is running
ros2 node list | grep bridge

# Check bridge topics
ros2 topic list | grep cmd_vel

# Verify message publishing
ros2 topic echo /cmd_vel
```

### No transforms published
```bash
# Check robot state publisher
ros2 node list | grep robot_state_publisher

# Verify TF output
ros2 run tf2_ros tf2_echo base_link drivewhl_l_link

# Check joint states
ros2 topic echo /joint_states
```

### Gazebo crashes or hangs
```bash
# Check logs
cat ~/.gazebo/server*.log

# Verify Gazebo installation
ign gazebo --version

# Test Gazebo separately
ign gazebo /opt/ros/humble/share/gazebo_ros/worlds/empty.world
```

### Keyboard teleop not working
```bash
# Ensure teleop_twist_keyboard is installed
sudo apt install ros-humble-teleop-twist-keyboard

# Run with proper parameters
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false
```

### For TurtleBot 3 teleop
```bash
# Install TurtleBot 3 teleop package
sudo apt install ros-humble-turtlebot3-teleop

# Run TurtleBot 3 keyboard control
ros2 run turtlebot3_teleop teleop_keyboard
```

### Poor simulation performance
1. Use headless mode: `headless:=true`
2. Increase max_step_size in world file
3. Reduce LiDAR update rate
4. Reduce Gazebo GUI quality

## Advanced Usage

### Record and Playback
```bash
# Record rosbag
ros2 bag record /cmd_vel /robot/odom /robot/imu /scan

# Playback rosbag
ros2 bag play rosbag2_folder/
```

### Launch with custom parameters
```bash
ros2 launch diff_drive_sim sim.launch.py \
  headless:=true \
  use_sim_time:=true \
  world_file:=/path/to/custom.sdf
```

### Integrate with Nav2
```bash
# After simulation is running, launch Nav2
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true
```

### Real robot deployment
The same `/cmd_vel` interface works with real hardware:
```bash
# Just replace sim.launch.py with real robot's hardware driver
ros2 launch my_robot_driver driver.launch.py
# Everything else remains the same!
```

## Performance Benchmarks

| Configuration | FPS | CPU Usage |
|--------------|-----|-----------|
| GUI + Full sensors | 60 | ~80% |
| GUI + Minimal sensors | 90+ | ~60% |
| Headless + Full sensors | 200+ | ~40% |
| Headless + Minimal sensors | 300+ | ~25% |

## File Reference

| File | Purpose |
|------|---------|
| `sim.launch.py` | Main simulation launcher (Gazebo + robot spawn + bridge) |
| `nav.launch.py` | Navigation stack launcher (AMCL + Nav2) |
| `diff_drive_robot.urdf` | Robot model (URDF format with xacro) |
| `diff_drive_robot.xacro` | Xacro macros and parameterized components |
| `diff_drive_robot.sdf` | Robot model (SDF format, auto-generated) |
| `default.sdf` | Default Gazebo world definition |
| `my_world.sdf` | Alternative Gazebo world |
| `bridge_config.yaml` | ROS 2 ↔ Gazebo topic bridge config |
| `nav_params.yaml` | Nav2 parameters (AMCL, planners, controllers) |
| `map.yaml` | Map metadata (origin, resolution, image reference) |
| `map.pgm` | Occupancy grid image |
| `rviz.config` | RViz2 visualization configuration |

## Documentation

- **QUICKSTART.md** - Get started in 5 minutes
- **CHANGE_LOG.md** - Version history and updates

## Common Workflows

### Workflow 1: Quick Test with Native ROS 2 Teleop
```bash
# Terminal 1: Start simulation
ros2 launch diff_drive_sim sim.launch.py

# Terminal 2: Start keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Workflow 2: Quick Test with TurtleBot 3 Teleop
```bash
# Terminal 1: Start simulation
ros2 launch diff_drive_sim sim.launch.py

# Terminal 2: Start TurtleBot 3 keyboard control
ros2 run turtlebot3_teleop teleop_keyboard
```

### Workflow 3: Development with Monitoring
```bash
# Terminal 1: Start simulation headless
ros2 launch diff_drive_sim sim.launch.py headless:=true

# Terminal 2: Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Monitor with RViz
rviz2
```

### Workflow 4: Testing & Recording
```bash
# Terminal 1: Start simulation
ros2 launch diff_drive_sim sim.launch.py

# Terminal 2: Start recording
ros2 bag record /cmd_vel /robot/odom /robot/imu /scan

# Terminal 3: Control robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 2: Stop recording (Ctrl+C)
```

### Workflow 5: Autonomous Navigation with AMCL
```bash
# Terminal 1: Start simulation
ros2 launch diff_drive_sim sim.launch.py

# Terminal 2: Start navigation stack (AMCL + path planning)
ros2 launch diff_drive_sim nav.launch.py

# Terminal 3: Open RViz for visualization and goal setting
rviz2

# In RViz:
# 1. Set initial pose with "2D Pose Estimate" tool
# 2. Set navigation goal with "2D Goal Pose" tool
# 3. Robot navigates autonomously to goal while avoiding obstacles
```

### Workflow 6: Full Integration Test
```bash
# Terminal 1: Start simulation
ros2 launch diff_drive_sim sim.launch.py headless:=true

# Terminal 2: Start navigation
ros2 launch diff_drive_sim nav.launch.py

# Terminal 3: Open RViz with full visualization
rviz2 -d $(ros2 pkg prefix diff_drive_sim)/share/diff_drive_sim/rviz/rviz.config

# Terminal 4: Manual teleoperation (if needed to help robot localize)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Contributing

To improve this package:
1. Test with your specific use case
2. Report issues with detailed reproduction steps
3. Submit pull requests with improvements
4. Update documentation accordingly

## References

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Fortress Documentation](https://gazebosim.org/)
- [ROS 2 Control](https://github.com/ros-controls/ros2_control)
- [TurtleBot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [DiffDrive Controller](https://github.com/ros-controls/ros2_controllers)

## License

This package is provided as-is for educational and research purposes.

## Support

For issues, questions, or contributions:
1. Check existing documentation
2. Review the troubleshooting section
3. Check ROS 2 and Gazebo forums

## Author
- Developed by Hrithik Dhongadi
