#!/usr/bin/env python3

import rospy
import math
import yaml
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

def load_origin_from_yaml(file_path, origin_name):
    """Load map origin information from YAML file"""
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            
        if 'saved_origins' not in data or origin_name not in data['saved_origins']:
            rospy.logerr(f"Origin '{origin_name}' not found.")
            return None
            
        return data['saved_origins'][origin_name]
    except Exception as e:
        rospy.logerr(f"Failed to load origin information: {e}")
        return None

def main():
    rospy.init_node('map_to_odom_tf_broadcaster')
    
    # Publication rate parameter
    publish_rate = rospy.get_param('~publish_rate', 10.0)
    origin_name = rospy.get_param('~origin_name', 'iam_field')
    origins_file = rospy.get_param('~origins_file', '')
    
    # Load map origin information
    if not origins_file:
        # Set default path
        origins_file = '$(find open_autonomous_tractor)/config/localization/saved_origins.yaml'
    
    origin_data = load_origin_from_yaml(origins_file, origin_name)
    if not origin_data:
        rospy.logerr("Unable to load map origin information.")
        return
    
    # Extract map origin coordinates
    origin_x = origin_data['origin_x']
    origin_y = origin_data['origin_y']
    
    # Simulation initial position parameters (set by pose_loader node)
    pose_x = rospy.get_param('/simulation/initial_pose/x', 0.0)
    pose_y = rospy.get_param('/simulation/initial_pose/y', 0.0)
    pose_yaw_rad = rospy.get_param('/simulation/initial_pose/yaw_rad', 0.0)
    
    # Calculate relative position in map frame
    relative_x = pose_x - origin_x
    relative_y = pose_y - origin_y
    
    rospy.loginfo(f"Map origin: x={origin_x}, y={origin_y}")
    rospy.loginfo(f"Robot initial position (absolute): x={pose_x}, y={pose_y}")
    rospy.loginfo(f"Robot initial position (map frame): x={relative_x}, y={relative_y}")
    
    # Set up TF broadcaster
    br = tf2_ros.TransformBroadcaster()
    tf_msg = TransformStamped()
    
    # Set up TF message
    tf_msg.header.frame_id = 'map'
    tf_msg.child_frame_id = 'odom'
    
    # Set transformation information - robot's initial position in map frame
    tf_msg.transform.translation.x = relative_x
    tf_msg.transform.translation.y = relative_y
    tf_msg.transform.translation.z = 0.0
    
    # Set rotation information - robot's initial orientation
    q = quaternion_from_euler(0, 0, pose_yaw_rad)
    tf_msg.transform.rotation.x = q[0]
    tf_msg.transform.rotation.y = q[1]
    tf_msg.transform.rotation.z = q[2]
    tf_msg.transform.rotation.w = q[3]
    
    # Set publication rate
    rate = rospy.Rate(publish_rate)
    
    # TF publication loop
    while not rospy.is_shutdown():
        tf_msg.header.stamp = rospy.Time.now()
        br.sendTransform(tf_msg)
        rate.sleep()

if __name__ == '__main__':
    main()