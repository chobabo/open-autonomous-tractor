# Field Boundary Visualization

## Overview
This work involves developing functionality to visualize field boundaries for agricultural tractor operations in RViz. The main objective is to read field boundary point information stored in the `field_boundaries.csv` file and visualize it in the ROS environment.

## Data Structure

### 1. Map Origin Information (`saved_origins.yaml`)
```yaml
saved_origins:
  # Location name: [plane rectangular coordinate system zone number, latitude, longitude, height, X, Y, Z, description]
  
  # Research institute (default)
  iam_field: 
    zone: 9
    latitude: 36.02233369
    longitude: 140.09684352
    height: 0.0
    origin_x: 2510.0
    origin_y: 23750.0
    origin_z: 0.0
    description: "ibaraki_iam_default"
  
  # Fukushima demonstration (not yet defined)
  fukushima_field:
    zone: 9
    latitude: 36.234567
    longitude: 140.234567
    height: 35.2
    origin_x: 23456.78
    origin_y: 34567.89
    origin_z: 0.0
    description: "fukushima_frei"
```

### 2. Field Boundary Point Information (`field_boundaries.csv`)
```
{field_name, P1, P2, P3, P4}
```
- P is a coordinate point stored in (origin_x, origin_y, origin_z) order
- Example: `ro1-a,24001.081798680512,2643.388529260177,0.0,23968.704049898268,2605.2875924846157,0.0,23946.852355489194,2623.820218836423,0.0,23979.252169692227,2661.9023936712183,0.0`

## Implementation Details

### 1. Folder Structure
A new folder structure has been added for field boundary visualization:

```
open_autonomous_tractor/
├── datasets/                 # Newly added folder
│   ├── iam_field/
│   │   └── field_boundaries.csv
│   └── fukushima_field/
│       └── field_boundaries.csv
├── scripts/
│   └── visualization/        # Newly added folder
│       ├── __init__.py
│       └── field_boundary_visualizer.py
├── launch/
│   └── visualization/        # Newly added folder
│       └── display_field_boundaries.launch
└── config/
    └── visualization/        # Newly added folder
        └── field_boundaries.rviz
```

### 2. Field Boundary Visualization Node (`field_boundary_visualizer.py`)
This Python node performs the following functions:
- Loads the origin information of the currently selected map from `saved_origins.yaml`
- Loads field boundary point information from `datasets/{map_name}/field_boundaries.csv`
- Creates MarkerArray messages to visualize field boundaries in RViz
- Converts plane rectangular coordinate system coordinates to ROS map frame coordinates (origin coordinate correction)
- Displays field name labels

### 3. RViz Visualization Settings
- Display each field in a different color
- Display the field name as text in the center of the field
- Display boundaries as closed loops

### 4. Launch File (`display_field_boundaries.launch`)
- Supports arguments for selecting various maps
- Loads default map information from `saved_origins.yaml`
- Option to choose whether to run RViz
- Runs the field boundary visualization node

## Coordinate System Conversion Issues
There are directional differences between the plane rectangular coordinate system and ROS coordinate system (REP 103), requiring appropriate conversion:

1. **Plane Rectangular Coordinate System**:
   - X-axis: East direction
   - Y-axis: North direction

2. **ROS Standard Coordinate System**:
   - X-axis: Forward direction
   - Y-axis: Left direction

Field boundary point coordinates are absolute positions from the origin of the plane rectangular coordinate system, so the following conversion is necessary for display in the map frame of RViz:

```python
# Plane rectangular coordinate system → ROS map frame conversion
point.x = x - origin_x  # Subtract the X coordinate of the origin
point.y = y - origin_y  # Subtract the Y coordinate of the origin
point.z = z - origin_z  # Subtract the Z coordinate of the origin
```

## Execution Method

1. **Prepare Field Boundary Data**:
   - Create `datasets/{map_name}/field_boundaries.csv` file

2. **Run the Node**:
   ```bash
   roslaunch open_autonomous_tractor display_field_boundaries.launch
   ```

3. **Run with a Different Map**:
   ```bash
   roslaunch open_autonomous_tractor display_field_boundaries.launch map_name:=fukushima_field
   ```

# Road Boundary Visualization

This package provides functionality to visualize road boundaries using ROS and RViz. It reads road boundary points and boundary line information from CSV files and displays them in RViz.

## Features

- Load road boundary point and boundary line data from CSV files
- Convert coordinates from plane rectangular coordinate system to RViz map frame coordinates
- Visualize road boundaries in RViz (line segments and point markers)
- Support switching between various fields/maps

## Data Format

### Road Boundary Points CSV (`road_boundary_points.csv`)

```
id, latitude, longitude, y, x
1, 36.0240736197222, 140.098826204444, 2703.52782243101, 23928.1724841952
...
```

- `id`: Boundary point identifier
- `latitude`, `longitude`: Latitude and longitude coordinates
- `y`, `x`: Plane rectangular coordinate system coordinates (in meters)

### Road Links CSV (`road_boundary_links.csv`)

```
start_id, end_id
1, 2
2, 3
...
```

- `start_id`: Start point ID (referencing ID in the boundary points CSV)
- `end_id`: End point ID (referencing ID in the boundary points CSV)

## Installation Method

1. Create dataset folder and save CSV files

```bash
mkdir -p ~/catkin_ws/src/open_autonomous_tractor/datasets/tsukuba_field
# Save road_boundary_points.csv and road_boundary_links.csv files in this folder
```

2. Add field information to the `saved_origins.yaml` file

```yaml
# Add Tsukuba field
tsukuba_field:
  zone: 9
  latitude: 36.0240736
  longitude: 140.0988262
  height: 0.0
  origin_x: 23928.0                   # Plane rectangular Y coordinate (m)
  origin_y: 2703.0                    # Plane rectangular X coordinate (m)
  origin_z: 0.0
  description: "tsukuba_road_boundaries"
  dataset_path: "datasets/tsukuba_field"  # Add dataset path
```

3. Grant execution permission

```bash
chmod +x ~/catkin_ws/src/open_autonomous_tractor/scripts/visualization/road_boundary_visualizer.py
```

## Usage

1. Run the road boundary visualization node

```bash
roslaunch open_autonomous_tractor display_road_boundaries.launch map_name:=tsukuba_field
```

2. RViz Configuration
   - Run RViz
   - Click "Add" button
   - Select "MarkerArray" for the `/road_boundaries` topic in the "By topic" tab

## Visualization Results

Visualization results are displayed in RViz as follows:
- Red lines: Road boundaries (LINE_LIST)
- Blue spheres: Road boundary points (SPHERE_LIST)

## File Description

- `road_boundary_visualizer.py`: ROS node for road boundary visualization
- `display_road_boundaries.launch`: Launch file for running the visualization node
- `saved_origins.yaml`: Map origin information configuration file

## Notes

- The coordinates in the road boundary point CSV file are relative positions based on the plane rectangular coordinate system, so conversion based on map origin information is necessary for display in RViz.
- When switching between different maps/fields, the `map_name` parameter must be set appropriately.

## Reference

- This code was developed based on the existing paddy fields boundary visualization code.
- Marker colors, sizes, and types can be modified as needed.

# 2D Nav Goal Pose Recorder

This is a feature to specify and record waypoints by clicking 2D Nav Goal in RViz. It sequentially saves and visualizes the poses (x, y, theta) of the specified locations.

## Features

- Record the pose (x, y, theta) of locations clicked with the 2D Nav Goal button in RViz
- Assign unique IDs to each pose and save them in a CSV file
- Visualize recorded pose information as text and directional arrows
- Automatically load and continue recording when a saved CSV file already exists

## Save Format

The CSV file format is as follows:
```
id,x,y,theta
0,12.345,6.789,0.843
1,13.456,7.890,1.234
...
```

## Execution Method

```bash
roslaunch open_autonomous_tractor nav_goal_recorder.launch
```

Specifying options:
```bash
# Specify save path and file name
roslaunch open_autonomous_tractor nav_goal_recorder.launch save_dir:=/path/to/save csv_filename:=my_poses.csv

# Specify coordinate frame
roslaunch open_autonomous_tractor nav_goal_recorder.launch frame_id:=odom
```

## Usage Instructions

1. Run RViz and tractor model
2. Run Nav Goal Recorder node
3. Click "2D Nav Goal" button in RViz
4. Click at the desired position and direction in 3D space
5. The clicked pose is automatically recorded and visualized

## Visualization Information

- Text marker: Displays ID, x, y, theta values for each pose
- Arrow marker: Indicates the direction (theta) of the pose

## Notes

- Pose information is saved as a CSV file in the specified path.
- Additional recording to an existing CSV file is possible.
- The spatial coordinate system uses the 'map' frame by default, which can be changed with the frame_id parameter.