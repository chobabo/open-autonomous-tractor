# Coordinate System and Position Configuration Files

The `/config/localization` directory contains configuration files necessary for position estimation and coordinate system conversion for autonomous tractors. This README document explains the role and key parameters of each file.

## File Structure

This directory contains the following main configuration files:

1. `coordinate_params.yaml`: Coordinate system conversion and TF settings parameters
2. `saved_origins.yaml`: Saved map origin information
3. `simulation_poses.yaml`: Simulation initial position information

## 1. coordinate_params.yaml

This file includes parameters needed for converting geographic coordinates (latitude/longitude) to a plane rectangular coordinate system and TF frame settings.

### Plane Rectangular Coordinate System (Japan)

Japan's plane rectangular coordinate system is a projected coordinate system divided into 19 origin points. Each origin has unique latitude and longitude and is used for accurate surveying and position calculation for specific regions.

```yaml
plane_rectangular_coordinates:
  default_zone: 9  # Zone corresponding to Ibaraki region (Japan)
  origin_points:
    # [latitude, longitude]
    - [0.0, 0.0]                   # 0: Origin (not used)
    - [33.0, 129.5]                # 1
    # ... omitted ...
    - [36.0, 139.83333333]         # 9: Ibaraki (default)
    # ... omitted ...
    - [26.0, 154.0]                # 19
```

### Coordinate Conversion Parameters

Ellipsoid parameters used for coordinate system conversion. Based on the GRS80 ellipsoid.

```yaml
conversion_params:
  m0: 0.9999                       # Scale factor
  a: 6378137.0                     # Semi-major axis
  F: 298.257222101                 # Reciprocal of flattening
```

### TF Frame Settings

Frame IDs and publishing frequency that constitute the robot's TF tree.

```yaml
tf_broadcaster:
  publish_rate: 10.0               # TF publishing rate (Hz)
  
  # Frame ID settings
  world_frame_id: "world"          # World coordinate system (origin: plane rectangular coordinate system origin)
  map_frame_id: "map"              # Map coordinate system (origin: work area origin)
  odom_frame_id: "odom"            # Odometry coordinate system (origin: robot initial position)
  base_frame_id: "base_footprint"  # Robot reference coordinate system
```

### GNSS/IMU Settings

Settings related to GNSS and IMU sensors.

```yaml
gnss_imu:
  gnss_topic: "/gnss/fix"          # GNSS position information topic
  imu_topic: "/imu/data"           # IMU data topic
  
  # Antenna offset (relative to base_link, in meters)
  antenna_offset_x: 0.0
  antenna_offset_y: 0.0
  antenna_offset_z: 0.0
```

### Map Origin Manager Settings

Settings for saving, loading, and managing map origins.

```yaml
map_origin_manager:
  saved_origins_file: "$(find open_autonomous_tractor)/config/localization/saved_origins.yaml"
  
  # Default map origin (coordinates in plane rectangular system, in meters)
  default_origin_x: 0.0
  default_origin_y: 0.0
  default_origin_z: 0.0
  
  # Service names
  set_origin_service: "set_map_origin"
  get_origin_service: "get_map_origin"
  save_origin_service: "save_map_origin"
  load_origin_service: "load_map_origin"
```

### Error Handling Settings

Error handling settings related to coordinate system conversion and position estimation.

```yaml
error_handling:
  use_fallback: true               # Whether to use fallback values when coordinate transformation fails
  gnss_timeout: 2.0                # GNSS signal loss detection timeout (seconds)
  
  # Transformation result filtering settings
  filter_outliers: true
  max_position_jump: 5.0           # Maximum position change (meters)
  max_orientation_jump: 0.5        # Maximum orientation change (radians)
```

## 2. saved_origins.yaml

This file stores map origin information for various work areas. Each origin has a unique name and location information.

### Map Origin Information Structure

Each map origin information is stored in the following structure:

```yaml
saved_origins:
  # Location name:
  origin_name:
    zone: coordinate_system_zone_number
    latitude: latitude
    longitude: longitude
    height: altitude
    origin_x: plane_rectangular_x_coordinate
    origin_y: plane_rectangular_y_coordinate
    origin_z: plane_rectangular_z_coordinate
    description: "description"
```

### Example Origin Information

```yaml
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
```

### Default Settings

Default map origin and last used origin in the current system.

```yaml
default_origin: "iam_field"
last_used_origin: "iam_field"
```

## 3. simulation_poses.yaml

This file defines the initial position and orientation of the robot in simulation mode. It can store initial positions for various work areas.

### Simulation Position Structure

Each simulation position is stored in the following structure:

```yaml
simulation_poses:
  location_name:
    map_zone: coordinate_system_zone_number
    pose:
      x: plane_rectangular_x_coordinate
      y: plane_rectangular_y_coordinate
      yaw_deg: direction_angle_degrees
    description: "description"
```

### Example Position Information

```yaml
# Ibaraki experiment field
iam_road:
  map_zone: 9
  pose:
    x: 2691.76      # Plane rectangular X coordinate (m)
    y: 23914.80     # Plane rectangular Y coordinate (m)
    yaw_deg: 50.0   # Direction angle (degrees)
  description: "ibaraki_iam_road"
```

## Coordinate System Conversion Flow

1. GNSS data (latitude/longitude)
2. → Plane rectangular coordinate system conversion (world frame)
3. → Map coordinate system conversion (map frame)
4. → Odometry coordinate system (odom frame)
5. → Robot coordinate system (base_footprint frame)

## Usage Examples

### Loading Simulation Position

```bash
roslaunch open_autonomous_tractor simulation.launch pose_name:=iam_road
```

### Setting a New Map Origin

```bash
rosservice call /set_map_origin "latitude: 36.02233369
longitude: 140.09684352
height: 0.0
description: 'new_field_origin'"
```

## TF Tree Structure

```
world → map → odom → base_footprint → base_link → sensors
```

- `world`: Plane rectangular coordinate system origin
- `map`: Work area origin
- `odom`: Robot initial position
- `base_footprint`: Robot floor center point
- `base_link`: Robot body center
- `sensors`: Various sensor frames
```