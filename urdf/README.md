# URDF Models

This directory contains URDF (Unified Robot Description Format) and XACRO (XML Macro) files that define the tractor model for the Open Autonomous Tractor project. These files describe the physical structure, visual appearance, and kinematic properties of the tractor.

## Directories

### common/
Contains common XACRO definitions used across different robot models:
- `common.xacro`: Defines common properties, macros, and materials

### robots/
Contains complete robot URDF specifications:
- `robot_tractor_model.urdf.xacro`: Main robot description that includes all components

### tractors/
Contains tractor-specific XACRO definitions:
- `tractor_model.urdf.xacro`: Detailed tractor model with links, joints, and physical properties

## File Structure

### XACRO Macros

XACRO (XML Macro) is used to simplify the creation of complex URDF files by allowing:
- Parametrization of values
- Reuse of common components
- Mathematical expressions
- Conditional inclusion of components

### Robot Model Components

The tractor model consists of:
- **Base link**: Main body of the tractor
- **Wheels**: Four wheels (front-left, front-right, rear-left, rear-right)
- **Steering joints**: For front wheel steering
- **Implement**: Agricultural attachment

## Key Parameters

The model uses parameters defined in configuration files:
- Dimensions (length, width, height, wheelbase, tread width)
- Wheel sizes (radius, width)
- Joint limits (steering angle ranges)
- Inertial properties (mass, inertia tensors)

## Usage

The URDF model is used in:
- **Visualization**: Display in RViz for debugging and monitoring
- **Simulation**: Physics-based simulation in Gazebo
- **Path Planning**: To account for tractor kinematics and dimensions
- **Control**: For accurate implementation of steering and movement

Load the URDF in a launch file using:

```xml
<param name="robot_description" command="$(find xacro)/xacro $(find open_autonomous_tractor)/urdf/robots/robot_tractor_model.urdf.xacro" />