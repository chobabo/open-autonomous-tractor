# Open Autonomous Tractor

An open-source ROS package for autonomous tractor simulation and development.

## Overview

This package provides a framework for simulating and developing autonomous navigation systems for agricultural tractors. It includes components for localization, path planning, visualization, and tractor model simulation.

The package is designed with a modular structure to support research and development in agricultural robotics, particularly focusing on autonomous tractors in field environments.

## Features

- **Tractor Model Simulation**: URDF-based tractor model with realistic physical parameters
- **Localization System**: Japanese Plane Rectangular Coordinate System for precise positioning
- **Path Planning**: Tools for recording and generating navigation paths
- **Visualization Tools**: Field and road boundary visualization for RViz
- **Teleoperation**: Simple keyboard-based teleop controls for manual operation

## Installation

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Python 3

### Building from Source

```bash
# Create a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/chobabo/open-autonomous-tractor.git

# Install dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
catkin_make

# Source the workspace
source ~/catkin_ws/devel/setup.bash
```

## License

This project is licensed under the [MIT License](LICENSE).