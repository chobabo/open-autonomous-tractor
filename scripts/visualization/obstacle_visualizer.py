#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
import os
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

class ObstacleVisualizer:
    def __init__(self):
        rospy.init_node('obstacle_visualizer', anonymous=True)
        
        # Load parameters
        self.update_rate = rospy.get_param('obstacle_visualizer/update_rate', 1.0)
        self.frame_id = rospy.get_param('obstacle_visualizer/frame_id', 'map')
        
        # Dataset path setup
        package_path = rospy.get_param('~package_path', os.path.expanduser('~/catkin_ws/src/open_autonomous_tractor'))
        self.obstacles_csv_path = os.path.join(package_path, 'datasets/obstacles/obstacles.csv')
        
        # Marker color parameters
        self.marker_color = {
            'r': rospy.get_param('obstacle_visualizer/marker_color/r', 1.0),
            'g': rospy.get_param('obstacle_visualizer/marker_color/g', 0.0),
            'b': rospy.get_param('obstacle_visualizer/marker_color/b', 0.0),
            'a': rospy.get_param('obstacle_visualizer/marker_color/a', 0.7)
        }
        
        # Publish obstacle markers
        self.marker_pub = rospy.Publisher('/visualization/obstacles', MarkerArray, queue_size=10)
        
        # Set timer (periodically publish markers)
        self.timer = rospy.Timer(rospy.Duration(self.update_rate), self.publish_markers)
        
        # Obstacle data
        self.obstacles = []
        # Initial loading
        self.load_obstacles()
        
        rospy.loginfo("Obstacle Visualizer initialized.")
        rospy.loginfo(f"Visualizing obstacles from: {self.obstacles_csv_path}")
    
    def load_obstacles(self):
        """Load obstacle information from CSV file"""
        if os.path.exists(self.obstacles_csv_path):
            try:
                df = pd.read_csv(self.obstacles_csv_path)
                self.obstacles = df.to_dict('records')
                rospy.loginfo(f"Loaded {len(self.obstacles)} obstacles from {self.obstacles_csv_path}")
            except Exception as e:
                rospy.logerr(f"Error loading obstacles CSV: {e}")
        else:
            rospy.logwarn(f"No obstacles file found at {self.obstacles_csv_path}")
            self.obstacles = []
        
    def publish_markers(self, event=None):
        """Publish obstacle markers to RViz"""
        # Periodically check obstacle file to reflect changes
        self.load_obstacles()
        
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(self.obstacles):
            marker = self.create_box_marker(obstacle, i)
            marker_array.markers.append(marker)
            
        # Delete markers for obstacles that no longer exist
        if hasattr(self, 'prev_marker_count') and self.prev_marker_count > len(self.obstacles):
            for i in range(len(self.obstacles), self.prev_marker_count):
                delete_marker = Marker()
                delete_marker.header.frame_id = self.frame_id
                delete_marker.header.stamp = rospy.Time.now()
                delete_marker.ns = "obstacles"
                delete_marker.id = i
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
        
        # Save previous marker count
        self.prev_marker_count = len(self.obstacles)
        
        self.marker_pub.publish(marker_array)
    
    def create_box_marker(self, obstacle, marker_id):
        """Create box marker"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set marker position and orientation
        marker.pose.position.x = obstacle['x']
        marker.pose.position.y = obstacle['y']
        marker.pose.position.z = obstacle['height'] / 2.0  # Set position at center, not at ground level
        
        # Set quaternion according to heading (theta)
        theta = obstacle['theta']
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(theta / 2.0)
        marker.pose.orientation.w = np.cos(theta / 2.0)
        
        # Set size
        marker.scale.x = obstacle['length']  # Length (forward direction)
        marker.scale.y = obstacle['width']   # Width (left-right direction)
        marker.scale.z = obstacle['height']  # Height
        
        # Set color
        marker.color.r = self.marker_color['r']
        marker.color.g = self.marker_color['g']
        marker.color.b = self.marker_color['b']
        marker.color.a = self.marker_color['a']
        
        return marker

    def run(self):
        """Run node"""
        rospy.loginfo("Obstacle Visualizer is running. Press Ctrl+C to terminate.")
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_visualizer = ObstacleVisualizer()
        obstacle_visualizer.run()
    except rospy.ROSInterruptException:
        pass