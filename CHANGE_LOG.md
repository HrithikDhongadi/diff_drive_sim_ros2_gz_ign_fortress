# CHANGE LOG 

| Date     | Changes                                             |
|:---------|:----------------------------------------------------|
|23-11-2025| Add URDF and XACRO files for differential drive robot with sensors |
|23-11-2025| Created `diff_drive_robot.urdf` to define the robot's structure, including links for the base, wheels, IMU, lidar, and camera.
|23-11-2025| Implemented `diff_drive_robot.xacro` to parameterize robot dimensions and inertia properties, enhancing maintainability.
|23-11-2025| Added Gazebo plugins for IMU, differential drive control, lidar, and camera sensors to facilitate simulation.
|23-11-2025| Established a default world in `default.sdf` with physics settings and a ground plane model.
|23-11-2025| Introduced a custom world in `my_world.sdf` featuring additional models and environmental settings for testing.
|23-11-2025| Added documentation for robot model and world files.
