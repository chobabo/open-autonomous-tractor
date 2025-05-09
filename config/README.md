# Configuration Files

This directory contains configuration files for the Open Autonomous Tractor project. These YAML files define parameters for localization, coordinate systems, and tractor model specifications.

## Directories

### localization/
Contains configuration files related to localization and coordinate systems:
- `coordinate_params.yaml`: Parameters for the Japanese Plane Rectangular Coordinate System
- `saved_origins.yaml`: Map origin information for different locations
- `simulation_poses.yaml`: Initial poses for simulation
- `README.md`: Detailed documentation on the localization system

### tractor_model/
Contains configuration files for the tractor model:
- `joint_limits.yaml`: Joint limit specifications for the tractor
- `tractor_model_params.yaml`: Physical parameters of the tractor (dimensions, weight, etc.)
- `tractor_model.rviz`: RViz configuration for visualizing the tractor

## Usage

These configuration files are loaded by the ROS nodes during runtime. You can modify these files to adjust the system behavior without recompiling the code.

Example using `rosparam` to load a configuration file:
```bash
rosparam load $(find open_autonomous_tractor)/config/tractor_model/tractor_model_params.yaml