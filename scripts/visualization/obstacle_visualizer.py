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
        
        # Set dataset path
        package_path = rospy.get_param('~package_path', os.path.expanduser('~/catkin_frei/src/frei_tractor'))
        self.obstacles_csv_path = os.path.join(package_path, 'datasets/obstacles/obstacles.csv')
        
        # Marker color parameters
        self.marker_color = {
            'r': rospy.get_param('obstacle_visualizer/marker_color/r', 0.0),
            'g': rospy.get_param('obstacle_visualizer/marker_color/g', 1.0),
            'b': rospy.get_param('obstacle_visualizer/marker_color/b', 0.0),
            'a': rospy.get_param('obstacle_visualizer/marker_color/a', 0.6)
        }
        
        # Publisher for obstacle markers
        self.marker_pub = rospy.Publisher('/visualization/obstacles', MarkerArray, queue_size=10)
        
        # Obstacle data and file tracking variables
        self.obstacles = []
        self.last_modified_time = 0
        
        # File check interval (default: check every 10 seconds)
        self.file_check_interval = rospy.get_param('~file_check_interval', 10.0)
        self.last_file_check = rospy.Time(0)
        
        # Initial load
        self.load_obstacles()
        
        # Timer setup (periodically publish markers)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.publish_markers)
        
        # Create and reuse markers once
        self.marker_array = MarkerArray()
        self.create_all_markers()
        
        rospy.loginfo("Obstacle Visualizer initialized.")
        rospy.loginfo(f"Visualizing obstacles from: {self.obstacles_csv_path}")
        rospy.loginfo(f"File check interval: {self.file_check_interval} seconds")
    
    def get_file_modified_time(self):
        """Get the last modification time of the file"""
        try:
            return os.path.getmtime(self.obstacles_csv_path)
        except OSError:
            return 0
    
    def should_reload_file(self):
        """Check if the file should be reloaded"""
        current_time = rospy.Time.now()
        
        # Check file modification time only at specified intervals
        if (current_time - self.last_file_check).to_sec() < self.file_check_interval:
            return False
        
        self.last_file_check = current_time
        current_modified_time = self.get_file_modified_time()
        
        # Check if file has been modified
        if current_modified_time > self.last_modified_time:
            self.last_modified_time = current_modified_time
            return True
        
        return False
    
    def load_obstacles(self):
        """Load obstacle information from CSV file"""
        if os.path.exists(self.obstacles_csv_path):
            try:
                df = pd.read_csv(self.obstacles_csv_path)
                self.obstacles = df.to_dict('records')
                self.last_modified_time = self.get_file_modified_time()
                rospy.loginfo(f"Loaded {len(self.obstacles)} obstacles from {self.obstacles_csv_path}")
                return True
            except Exception as e:
                rospy.logerr(f"Error loading obstacles CSV: {e}")
                return False
        else:
            rospy.logwarn(f"No obstacles file found at {self.obstacles_csv_path}")
            self.obstacles = []
            return False
    
    def create_all_markers(self):
        """Pre-create all obstacle markers"""
        self.marker_array = MarkerArray()
        
        for i, obstacle in enumerate(self.obstacles):
            marker = self.create_box_marker(obstacle, i)
            self.marker_array.markers.append(marker)
        
        # Create deletion markers for markers that no longer exist
        if hasattr(self, 'prev_marker_count') and self.prev_marker_count > len(self.obstacles):
            for i in range(len(self.obstacles), self.prev_marker_count):
                delete_marker = Marker()
                delete_marker.header.frame_id = self.frame_id
                delete_marker.header.stamp = rospy.Time.now()
                delete_marker.ns = "obstacles"
                delete_marker.id = i
                delete_marker.action = Marker.DELETE
                self.marker_array.markers.append(delete_marker)
        
        self.prev_marker_count = len(self.obstacles)
    
    def publish_markers(self, event=None):
        """Publish obstacle markers to RViz"""
        # Check if file has been modified and reload only if necessary
        if self.should_reload_file():
            rospy.loginfo("Obstacle file modified, reloading...")
            if self.load_obstacles():
                self.create_all_markers()
        
        # Publish pre-created marker array (only update timestamps)
        for marker in self.marker_array.markers:
            if marker.action != Marker.DELETE:
                marker.header.stamp = rospy.Time.now()
        
        self.marker_pub.publish(self.marker_array)
    
    def create_box_marker(self, obstacle, marker_id):
        """Create box marker for obstacle"""
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
        marker.pose.position.z = obstacle['height'] / 2.0  # Set at center height, not ground
        
        # Set quaternion based on theta (orientation)
        theta = obstacle['theta']
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(theta / 2.0)
        marker.pose.orientation.w = np.cos(theta / 2.0)
        
        # Set size
        marker.scale.x = obstacle['length']  # Length (forward direction)
        marker.scale.y = obstacle['width']   # Width (lateral direction)
        marker.scale.z = obstacle['height']  # Height
        
        # Set color
        marker.color.r = self.marker_color['r']
        marker.color.g = self.marker_color['g']
        marker.color.b = self.marker_color['b']
        marker.color.a = self.marker_color['a']
        
        return marker

    def run(self):
        """Run the node"""
        rospy.loginfo("Obstacle Visualizer is running. Press Ctrl+C to terminate.")
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_visualizer = ObstacleVisualizer()
        obstacle_visualizer.run()
    except rospy.ROSInterruptException:
        pass