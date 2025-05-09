# ROS Tractor Simulation Configuration

This document describes the simulation configuration of the `open_autonomous_tractor` package. It focuses specifically on the tractor simulation environment setup through the `simulation.launch` file.

## Simulation Overview

This simulation is designed to simulate the movement of an actual tractor in a ROS environment, representing the relationship between the real world and the robot through a TF tree. The TF tree structure used in the simulation is as follows:

```
world → map → odom → base_footprint → base_link → sensors
```

Meaning of each frame:
- `world`: Origin of the plane rectangular coordinate system
- `map`: Origin of the work area (based on saved location information)
- `odom`: Robot's initial position frame (based on simulation position data)
- `base_footprint`: Robot's real-time position frame
- `base_link`: Center frame of the robot body
- `sensors`: Various sensor frames

## Key Components

### 1. Parameter Settings

The simulation uses the following key parameters:

- `pose_name`: Robot initial position name (default: "iam_road")
- `poses_file`: Robot position information file path
- `origin_name`: Map origin name (default: "iam_field")
- `origins_file`: Map origin information file path
- `tractor_model`: Tractor model type (default: "tractor_model")

### 2. Related YAML Files Loading

The simulation loads the following YAML files:

- **coordinate_params.yaml**: Plane rectangular coordinate system parameters (zones, origin definitions, etc.)
- **tractor_model_params.yaml**: Tractor model parameters (dimensions, steering limits, etc.)
- **saved_origins.yaml**: Saved map origin information (location, description, etc.)
- **simulation_poses.yaml**: Simulation initial position information

### 3. Main Nodes

The simulation runs the following main nodes:

#### 3.1 simul_load_pose.py
- **Role**: Load simulation initial position information and set parameters
- **Function**: Load the robot's initial position information from `simulation_poses.yaml` and set it as ROS parameters

#### 3.2 simul_world_to_map_tf_broadcaster.py
- **Role**: Publish `world` → `map` TF transformation
- **Function**: Load map origin information and publish transformation information from the plane rectangular coordinate system origin (`world`) to the map origin (`map`)

#### 3.3 simul_map_to_odom_tf_broadcaster.py
- **Role**: Publish `map` → `odom` TF transformation
- **Function**: Use the robot's initial position information and map origin information to publish relative transformation information from the map origin to the robot's initial position

#### 3.4 simul_tractor_odometry.py
- **Role**: Publish `odom` → `base_footprint` TF transformation and odometry messages
- **Function**: Calculate odometry based on the robot's velocity and steering angle information and update position and orientation

## Usage

### Running the Simulation

To run the simulation with default settings:

```bash
roslaunch open_autonomous_tractor simulation.launch

roslaunch open_autonomous_tractor display_rviz_tractor_model.launch

roslaunch open_autonomous_tractor simul_tractor_teleop.py
```

To run with specific position and map origin:

```bash
roslaunch open_autonomous_tractor simulation.launch pose_name:=iam_road origin_name:=iam_field
```

To use a different tractor model:

```bash
roslaunch open_autonomous_tractor simulation.launch tractor_model:=tractor_model
```

### Robot Control

The robot is controlled by subscribing to the following topics:

- `/cmd_vel` (geometry_msgs/Twist): Control the linear velocity of the robot
- `/steering_angle` (std_msgs/Float64): Control the steering angle of the robot

### Checking the TF Tree

To check the TF tree of the running simulation:

```bash
rosrun tf view_frames
```

## Notes

- This simulation assumes that the robot has no wheel odometry and calculates odometry based on velocity and steering angle information.
- Tractor model parameters are automatically loaded from the corresponding model's YAML file, so there is no need to manually adjust parameters when changing models.
- Coordinate transformation is based on the Japanese plane rectangular coordinate system (default zone: 9 - Ibaraki).