#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node('map_to_odom_tf_broadcaster')
    
    # Publishing rate parameter
    publish_rate = rospy.get_param('~publish_rate', 10.0)
    
    # TF broadcaster setup
    br = tf2_ros.TransformBroadcaster()
    tf_msg = TransformStamped()
    
    # TF message setup - Identity transform (map == odom)
    tf_msg.header.frame_id = 'map'
    tf_msg.child_frame_id = 'odom'
    
    # Identity transformation setup (map and odom are identical)
    tf_msg.transform.translation.x = 0.0
    tf_msg.transform.translation.y = 0.0
    tf_msg.transform.translation.z = 0.0
    
    # Identity rotation (no rotation)
    tf_msg.transform.rotation.x = 0.0
    tf_msg.transform.rotation.y = 0.0
    tf_msg.transform.rotation.z = 0.0
    tf_msg.transform.rotation.w = 1.0
    
    # Publishing rate setup
    rate = rospy.Rate(publish_rate)
    
    rospy.loginfo("Broadcasting map->odom identity transform")
    rospy.loginfo("This means map and odom frames are identical")
    rospy.loginfo(f"Publishing at {publish_rate} Hz")
    
    # TF publishing loop
    while not rospy.is_shutdown():
        tf_msg.header.stamp = rospy.Time.now()
        br.sendTransform(tf_msg)
        rate.sleep()

if __name__ == '__main__':
    main()