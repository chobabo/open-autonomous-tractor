#!/usr/bin/env python3

import rospy
import yaml
import os
import sys
from pprint import pprint

# Add path to import modules from parent directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from localization.coordinate_converter import CoordinateConverter

def load_yaml(file_path):
    """Load YAML file"""
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")
        return None

def main():
    # Initialize ROS node
    rospy.init_node('coordinate_conversion_sample', anonymous=True)
    
    # Get package path
    package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
    
    # Configuration file paths
    params_file = os.path.join(package_path, 'config/localization/coordinate_params.yaml')
    origins_file = os.path.join(package_path, 'config/localization/saved_origins.yaml')
    
    # Check if files exist
    if not os.path.exists(params_file):
        rospy.logerr(f"File not found: {params_file}")
        return
    if not os.path.exists(origins_file):
        rospy.logerr(f"File not found: {origins_file}")
        return
    
    # Load configuration files
    params = load_yaml(params_file)
    origins = load_yaml(origins_file)
    
    if not params:
        rospy.logerr(f"Cannot load file {params_file}")
        return
    if not origins:
        rospy.logerr(f"Cannot load file {origins_file}")
        return
    
    rospy.loginfo("Configuration files loaded successfully")
    
    # Extract conversion parameters
    conversion_params = params.get('conversion_params', {})
    
    # Initialize coordinate converter
    converter = CoordinateConverter(conversion_params)
    
    # Test plane rectangular coordinate system origin (zone 9)
    zone = 9
    origin_point = converter.get_origin_by_zone(zone)
    
    if not origin_point:
        rospy.logerr(f"Cannot get origin for zone {zone}")
        return
    
    origin_lat, origin_lon = origin_point
    
    # Test coordinates (plane rectangular coordinate system)
    test_x = 2510.0
    test_y = 23750.0
    
    print("\n" + "="*60)
    print(f"Test Coordinate Conversion (Zone {zone})")
    print("="*60)
    
    # Print origin information
    print(f"Origin (Zone {zone}):")
    print(f"  Latitude: {origin_lat:.8f}°")
    print(f"  Longitude: {origin_lon:.8f}°")
    print()
    
    # Plane rectangular coordinate system -> Latitude/Longitude conversion
    lat, lon = converter.calc_lat_lon(test_x, test_y, origin_lat, origin_lon)
    
    print(f"Plane rectangular coordinate system -> Latitude/Longitude:")
    print(f"  Input: X={test_x:.2f}m, Y={test_y:.2f}m")
    print(f"  Output: Latitude={lat:.8f}°, Longitude={lon:.8f}°")
    print()
    
    # Latitude/Longitude -> Plane rectangular coordinate system conversion (inverse verification)
    back_x, back_y = converter.calc_xy(lat, lon, origin_lat, origin_lon)
    
    print(f"Latitude/Longitude -> Plane rectangular coordinate system (inverse conversion):")
    print(f"  Input: Latitude={lat:.8f}°, Longitude={lon:.8f}°")
    print(f"  Output: X={back_x:.2f}m, Y={back_y:.2f}m")
    print()
    
    # Calculate error
    x_error = abs(test_x - back_x)
    y_error = abs(test_y - back_y)
    
    print(f"Conversion Error:")
    print(f"  X Error: {x_error:.8f}m")
    print(f"  Y Error: {y_error:.8f}m")
    print("="*60)
    
    # Additional: Print list of origins
    print("\nList of Origins:")
    for i, point in enumerate(converter.get_origin_points()):
        if i > 0:  # 0 is not used
            print(f"  Zone {i}: Latitude={point[0]:.8f}°, Longitude={point[1]:.8f}°")
    
    # Print saved origin information (if available)
    if origins and 'saved_origins' in origins:
        print("\nSaved Origin Information:")
        for name, info in origins['saved_origins'].items():
            print(f"  {name}:")
            print(f"    Zone: {info.get('zone')}")
            print(f"    Latitude: {info.get('latitude'):.8f}°")
            print(f"    Longitude: {info.get('longitude'):.8f}°")
            print(f"    Description: {info.get('description', 'None')}")
    
    print("\nSample execution completed")

if __name__ == '__main__':
    main()