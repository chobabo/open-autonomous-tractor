#!/usr/bin/env python3

import rospy
import yaml
import os
import sys
import math
from tf.transformations import quaternion_from_euler

def load_pose_from_yaml(file_path, pose_name):
    """Load specific position information from YAML file"""
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            
        if 'simulation_poses' not in data or pose_name not in data['simulation_poses']:
            rospy.logerr(f"Position '{pose_name}' not found.")
            return None
            
        return data['simulation_poses'][pose_name]
    except Exception as e:
        rospy.logerr(f"Failed to load position information: {e}")
        return None

def main():
    rospy.init_node('simulation_pose_loader')
    
    # Get parameters
    pose_name = rospy.get_param('~pose_name', 'ibaraki_field1')
    poses_file = rospy.get_param('~poses_file', '')
    use_custom_pose = rospy.get_param('~use_custom_pose', False)
    
    if use_custom_pose:
        # Use custom position
        pose_x = rospy.get_param('~custom_pose_x', 0.0)
        pose_y = rospy.get_param('~custom_pose_y', 0.0)
        pose_yaw_deg = rospy.get_param('~custom_pose_yaw_deg', 0.0)
        map_zone = rospy.get_param('~custom_map_zone', 9)
        
        rospy.loginfo(f"Using custom position: x={pose_x}, y={pose_y}, yaw={pose_yaw_deg}°, zone={map_zone}")
    else:
        # Load position from YAML file
        if not poses_file:
            poses_file = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                'config/localization/simulation_poses.yaml'
            )
        
        pose_data = load_pose_from_yaml(poses_file, pose_name)
        if not pose_data:
            rospy.logerr("Unable to load simulation position.")
            return
            
        pose_x = pose_data['pose']['x']
        pose_y = pose_data['pose']['y']
        pose_yaw_deg = pose_data['pose']['yaw_deg']
        map_zone = pose_data['map_zone']
        
        rospy.loginfo(f"Loaded position '{pose_name}': x={pose_x}, y={pose_y}, yaw={pose_yaw_deg}°, zone={map_zone}")
    
    # Set global parameters
    rospy.set_param('/simulation/initial_pose/x', pose_x)
    rospy.set_param('/simulation/initial_pose/y', pose_y)
    rospy.set_param('/simulation/initial_pose/yaw_deg', pose_yaw_deg)
    rospy.set_param('/simulation/initial_pose/yaw_rad', math.radians(pose_yaw_deg))
    rospy.set_param('/simulation/map_zone', map_zone)
    
    # Additional configuration code here (e.g., TF publishing, etc.)
    
    rospy.loginfo("Simulation initial position setup completed")
    
if __name__ == '__main__':
    main()