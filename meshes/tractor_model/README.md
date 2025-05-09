# Mesh Files

This directory contains 3D mesh files for the Open Autonomous Tractor project. These files define the visual appearance and collision properties of the tractor model in simulation and visualization.

## Directories

### tractor_model/
Contains mesh files for the default tractor model:

#### collision/
Low-resolution meshes used for physics calculations:
- `body.stl`: Main tractor body
- `front_left.stl`: Front left wheel
- `front_right.stl`: Front right wheel
- `implement.stl`: Agricultural implement
- `rear_left.stl`: Rear left wheel
- `rear_right.stl`: Rear right wheel

#### visual/
Detailed meshes used for visualization:
- `body.stl`: Main tractor body
- `front_left.stl`: Front left wheel
- `front_right.stl`: Front right wheel
- `implement.stl`: Agricultural implement
- `rear_left.stl`: Rear left wheel
- `rear_right.stl`: Rear right wheel

## File Format

All meshes are stored in STL (STereoLithography) format, which is widely supported by 3D modeling software and ROS visualization tools. STL files describe only the surface geometry of three-dimensional objects without color, texture, or other attributes.

## Usage in URDF

These mesh files are referenced in the URDF model definitions using the package URL scheme:

```xml
<mesh filename="package://open_autonomous_tractor/meshes/tractor_model/visual/body.stl" scale="1 1 1"/>