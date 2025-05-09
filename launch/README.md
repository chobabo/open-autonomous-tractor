# Launch Files

This directory contains ROS launch files for the Open Autonomous Tractor project. Launch files provide a convenient way to start multiple ROS nodes simultaneously with the appropriate parameters.

## Directories

### localization/
Contains launch files related to localization and coordinate transformation:
- `simulation.launch`: Sets up the simulation environment with TF frames and odometry
- `README.md`: Detailed documentation on the simulation setup

### planning/
Contains launch files for path planning and navigation:
- `global_path_generator.launch`: Generates a global path from recorded poses
- `README.md`: Documentation on path planning components

### tractor_model/
Contains launch files for visualizing the tractor model:
- `display_rviz_tractor_model.launch`: Displays the tractor model in RViz

### visualization/
Contains launch files for different visualization tools:
- `display_field_boundaries.launch`: Visualizes field boundaries in RViz
- `display_road_boundaries.launch`: Visualizes road boundaries in RViz
- `nav_goal_recorder.launch`: Records navigation goals from RViz 2D Pose Tool
- `README.md`: Documentation on visualization tools

## Usage Examples

### Starting the Simulation

```bash
roslaunch open_autonomous_tractor simulation.launch