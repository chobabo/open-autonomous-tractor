# Datasets

This directory contains datasets for the Open Autonomous Tractor project. These datasets include field boundaries, road information, and navigation waypoints that are used for testing and simulation.

## Directories

### iam_field/
Contains field and road data for the IAM (Ibaraki Agricultural Model) test field:
- `field_boundaries.csv`: Field boundary vertex coordinates
- `road_boundary_links.csv`: Road segment connections (start and end point IDs)
- `road_boundary_points.csv`: Road boundary vertex coordinates

### nav_goals/
Contains navigation goal and path data:
- `global_path.csv`: Generated global path with interpolated waypoints
- `recorded_poses.csv`: Manually recorded navigation poses (positions and orientations)

## File Formats

### Field Boundaries CSV
Field boundaries are stored in CSV format with the following structure:
```
field_name,P1x,P1y,P1z,P2x,P2y,P2z,...
```
Where:
- `field_name`: Name of the field area
- `P1x,P1y,P1z`: Coordinates of the first vertex (X, Y, Z)
- `P2x,P2y,P2z`: Coordinates of the second vertex, and so on

### Road Boundary Points CSV
Road boundary points are stored with the format:
```
id,latitude,longitude,y,x
```
Where:
- `id`: Unique identifier for the point
- `latitude,longitude`: Geodetic coordinates
- `y,x`: Plane rectangular coordinates (in meters)

### Road Boundary Links CSV
Road links connect boundary points with the format:
```
start_id,end_id
```
Where:
- `start_id`: ID of the starting point
- `end_id`: ID of the ending point

### Navigation CSV Files
Navigation files use the format:
```
id,x,y,theta
```
Where:
- `id`: Sequential identifier
- `x,y`: Position coordinates (in meters)
- `theta`: Orientation angle (in radians)

## Adding Custom Datasets

To add a custom dataset:
1. Create a new directory with your location name (e.g., `your_field/`)
2. Add the required CSV files following the formats described above
3. Update the `saved_origins.yaml` in the `config/localization/` directory with your field's origin information
```