# CHANGE LOG 

| Date     | Changes                                             |
|:---------|:----------------------------------------------------|
|23-11-2025| Add URDF and XACRO files for differential drive robot with sensors |
|23-11-2025| Created `diff_drive_robot.urdf` to define the robot's structure, including links for the base, wheels, IMU, lidar, and camera. |
|23-11-2025| Implemented `diff_drive_robot.xacro` to parameterize robot dimensions and inertia properties, enhancing maintainability. |
|23-11-2025| Added Gazebo plugins for IMU, differential drive control, lidar, and camera sensors to facilitate simulation. |
|23-11-2025| Established a default world in `default.sdf` with physics settings and a ground plane model. |
|23-11-2025| Introduced a custom world in `my_world.sdf` featuring additional models and environmental settings for testing. |
|23-11-2025| Added documentation for robot model and world files. |
|24-11-2025| Integrated navigation stack with configuration files for differential drive robot. |
|25-11-2025| Created slam.launch.py for real-time SLAM map creation with slam_toolbox |
|25-11-2025| SLAM uses online async mode for continuous mapping as robot explores |
|25-11-2025| README: Added comprehensive Section 5 - SLAM Map Creation guide |
|25-11-2025| README: Added SLAM topics to ROS 2 Topics reference |
|25-11-2025| README: Updated Launch Configuration with slam.launch.py details |
|25-11-2025| README: Added slam.launch.py to File Reference table |
|25-11-2025| README: Added 2 new workflows (Workflow 7 & 8) for SLAM and SLAM→Navigation pipeline |
|25-11-2025| README: Added map saving instructions and map configuration details |
|25-11-2025| Updated References section with Nav2 and SLAM Toolbox documentation |
|25-11-2025| Fixed minor bugs in URDF and XACRO files for better compatibility with Gazebo and ROS 2. |
|25-11-2025| Add a closed world environment with obstacles for navigation and SLAM testing. |
|25-11-2025| README: Added Available Maps section documenting all 4 maps (default, my_map2, closed_map, closed_map_v2) |
|25-11-2025| README: Enhanced SLAM Parameters section with detailed mapper_params_online_async.yaml documentation |
|25-11-2025| README: Updated Package Structure to include mapper_params_online_async.yaml and all maps |
|25-11-2025| README: Updated File Reference table with all map files and mapper_params_online_async.yaml |
|25-11-2025| README: Added serialize_map documentation for pose graph serialization |
|25-11-2025| README: Added "Serialize Map for Future Mapping Sessions" subsection with examples |
|25-11-2025| README: Added Workflow 9 - Advanced SLAM with pose graph serialization (closed_map_v2 example) |
|25-11-2025| Documented benefits of pose graph serialization for continuity and optimization |  
