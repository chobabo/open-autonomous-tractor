# Global Path Generator

This ROS node reads a series of poses (position and orientation) stored in a CSV file to generate a global path. It dynamically creates straight or curved paths based on angle differences, and the generated path can be visualized in RViz or saved as a CSV file.

## Key Features

- **Pose-based Global Path Generation**: Load pose data in the format of `id, x, y, theta` from a CSV file to generate a global path
- **Automatic Straight/Curve Determination**: Automatically generate straight or curved segments based on theta (direction angle) difference
  - Create a straight line if the theta difference is less than a threshold (default threshold: 0.2 radians ≈ 11.46 degrees)
  - Create a Bezier curve if the theta difference is greater than the threshold
- **Configurable Path Spacing**: Generate path points with configurable spacing (default: 0.1m)
- **Visualization**: Visualize the generated path and original poses in RViz
- **CSV Saving**: Save the generated global path as a CSV file

## Algorithm Details

### Path Generation Algorithm

The global path generator works as follows:

1. Load pose data from a CSV file
2. For each consecutive pair of poses:
   - Calculate the distance and theta difference between the two poses
   - **Straight Segment** (theta difference ≤ threshold): 
     - Use linear interpolation to generate x, y, theta values
   - **Curved Segment** (theta difference > threshold): 
     - Use cubic Bezier curves to create smooth curves
     - Form natural curves using control points based on theta direction
3. Generate points at the specified spacing for all segments

### Special Angle Processing Features

- **Angle Normalization**: Normalize all angles to be between -π and π
- **Angle Interpolation**: Interpolate between two angles along the shortest path (e.g., move only 20° between 350° and 10°)
- **Quaternion Normalization**: Always use normalized quaternions to represent orientation (prevents RViz warnings)

## Usage

### Basic Execution

```bash
roslaunch open_autonomous_tractor global_path_generator.launch
```

### Execution with Custom Parameters

```bash
roslaunch open_autonomous_tractor global_path_generator.launch \
  csv_file:=/path/to/input/poses.csv \
  output_csv_file:=/path/to/output/global_path.csv \
  path_spacing:=0.2 \
  theta_threshold:=0.3
```

### How to Save Global Path CSV

#### 1. Automatic Saving (at node start)

```bash
roslaunch open_autonomous_tractor global_path_generator.launch auto_save:=true
```

#### 2. ROS Service Call (using default filename)

```bash
rosservice call /save_global_path "{}"
```

#### 3. Topic Publishing (with custom filename)

```bash
rostopic pub /save_path_to_file std_msgs/String "data: '/path/to/save/custom_path.csv'" -1
```

#### 4. Using the Client Script

```bash
# Save with default filename
rosrun open_autonomous_tractor save_global_path_client.py

# Save with custom filename
rosrun open_autonomous_tractor save_global_path_client.py -f /path/to/save/custom_path.csv
```

## Parameter Description

| Parameter | Description | Default Value |
|----------|------|--------|
| `csv_file` | Input CSV file path (pose data) | `recorded_poses.csv` |
| `output_csv_file` | Output CSV file path (global path storage) | `global_path.csv` |
| `auto_save` | Whether to save automatically at node start | `false` |
| `path_spacing` | Point spacing in the generated path (meters) | `0.1` |
| `theta_threshold` | Threshold for straight/curve determination (radians) | `0.2` (≈11.46 degrees) |
| `frame_id` | Frame ID for the path | `map` |
| `publish_rate` | Path publishing rate (Hz) | `1.0` |

## Input/Output CSV File Format

### Input CSV File (Pose Data)

```
id,x,y,theta
1,0.0,0.0,0.0
2,1.0,0.0,0.0
3,2.0,0.2,0.1
...
```

### Output CSV File (Global Path)

```
id,x,y,theta
1,0.0,0.0,0.0
2,0.1,0.0,0.0
3,0.2,0.0,0.0
...
```
