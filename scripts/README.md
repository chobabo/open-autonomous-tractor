# Scripts

This directory contains Python scripts for the Open Autonomous Tractor project. These scripts implement the core functionality for localization, path planning, control, and visualization.

## Directories

### control/
Contains scripts related to tractor motion control and path following:
- Currently contains initialization file for the Python package structure

### detection/
Contains scripts for obstacle detection and environmental perception:
- Currently contains initialization file for the Python package structure

### localization/
Contains scripts for position estimation and coordinate transformation:
- `coordinate_converter.py`: Implements conversion between geographic and plane rectangular coordinates
- `__init__.py`: Initialization file for the Python package structure

### planning/
Contains scripts for path planning and trajectory generation:
- `global_path_generator.py`: Generates smooth paths from waypoints
- `__init__.py`: Initialization file for the Python package structure

### sample/
Contains example scripts demonstrating the usage of various modules:
- `coordinate_conversion_sample.py`: Demonstrates coordinate conversion functionality

### simulation/
Contains scripts for simulating the tractor's behavior in a virtual environment:
- `simul_load_pose.py`: Loads initial pose information from YAML files
- `simul_map_to_odom_tf_broadcaster.py`: Broadcasts map to odom transform
- `simul_steering_joint_updater.py`: Updates steering joint values
- `simul_tractor_odometry.py`: Simulates tractor odometry
- `simul_tractor_teleop.py`: Keyboard teleoperation for the tractor
- `simul_world_to_map_tf_broadcaster.py`: Broadcasts world to map transform

### visualization/
Contains scripts for visualizing tractor data and environment features:
- `field_boundary_visualizer.py`: Visualizes field boundaries in RViz
- `nav_goal_recorder.py`: Records navigation goals from RViz 2D Pose Tool
- `road_boundary_visualizer.py`: Visualizes road boundaries in RViz
- `__init__.py`: Initialization file for the Python package structure

## Python Version

All scripts are written for Python 3 and include the appropriate shebang line:
```python
#!/usr/bin/env python3