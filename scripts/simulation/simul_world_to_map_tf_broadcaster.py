#!/usr/bin/env python3

import rospy
import yaml
import tf
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

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
    rospy.init_node('world_to_map_tf_broadcaster')
    
    # Get parameters
    origin_name = rospy.get_param('~origin_name', 'iam_field')
    origins_file = rospy.get_param('~origins_file', '')
    publish_rate = rospy.get_param('~publish_rate', 10.0)
    
    # Load origin information
    if not origins_file:
        # Set default path
        origins_file = '$(find open_autonomous_tractor)/config/localization/saved_origins.yaml'
    
    origin_data = load_origin_from_yaml(origins_file, origin_name)
    if not origin_data:
        rospy.logerr("Unable to load map origin information.")
        return
    
    # Extract origin coordinates
    origin_x = origin_data['origin_x']
    origin_y = origin_data['origin_y']
    origin_z = origin_data['origin_z']
    
    rospy.loginfo(f"Loaded origin '{origin_name}': x={origin_x}, y={origin_y}, z={origin_z}")
    
    # Set up TF broadcaster
    br = tf2_ros.TransformBroadcaster()
    tf_msg = TransformStamped()
    
    # Set up TF message
    tf_msg.header.frame_id = 'world'
    tf_msg.child_frame_id = 'map'
    
    # Set transformation information
    tf_msg.transform.translation.x = origin_x
    tf_msg.transform.translation.y = origin_y
    tf_msg.transform.translation.z = origin_z
    
    # No rotation (simple translation only)
    tf_msg.transform.rotation.x = 0.0
    tf_msg.transform.rotation.y = 0.0
    tf_msg.transform.rotation.z = 0.0
    tf_msg.transform.rotation.w = 1.0
    
    # Set publication rate
    rate = rospy.Rate(publish_rate)
    
    # TF publication loop
    while not rospy.is_shutdown():
        tf_msg.header.stamp = rospy.Time.now()
        br.sendTransform(tf_msg)
        rate.sleep()

if __name__ == '__main__':
    main()