#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
import os
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class ObstacleManager:
    def __init__(self):
        rospy.init_node('obstacle_manager', anonymous=True)
        
        # Use parameters loaded from YAML file
        self.default_width = rospy.get_param('obstacle_manager/default_width', 1.0)
        self.default_length = rospy.get_param('obstacle_manager/default_length', 1.0)
        self.default_height = rospy.get_param('obstacle_manager/default_height', 1.0)
        
        # Marker color parameters
        self.marker_color = {
            'r': rospy.get_param('obstacle_manager/marker_color/r', 1.0),
            'g': rospy.get_param('obstacle_manager/marker_color/g', 0.0),
            'b': rospy.get_param('obstacle_manager/marker_color/b', 0.0),
            'a': rospy.get_param('obstacle_manager/marker_color/a', 0.7)
        }
        
        # Other parameters
        self.update_rate = rospy.get_param('obstacle_manager/update_rate', 0.5)
        self.frame_id = rospy.get_param('obstacle_manager/frame_id', 'map')
        
        # Dataset path setup (following existing path pattern)
        package_path = rospy.get_param('~package_path', os.path.expanduser('~/catkin_ws/src/open_autonomous_tractor'))
        self.obstacles_dir = os.path.join(package_path, 'datasets/obstacles')
        self.obstacles_csv_path = os.path.join(self.obstacles_dir, 'obstacles.csv')
        
        # Create directory
        os.makedirs(self.obstacles_dir, exist_ok=True)
        
        # Obstacle data
        self.obstacles = []
        self.load_obstacles()
        
        # Subscribe to 2D Nav Goal
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Publish obstacle markers
        self.marker_pub = rospy.Publisher('/visualization/obstacles', MarkerArray, queue_size=10)
        
        # Set timer (periodically publish markers)
        self.timer = rospy.Timer(rospy.Duration(self.update_rate), self.publish_markers)
        
        rospy.loginfo("Obstacle Manager initialized with following parameters:")
        rospy.loginfo(f"- Default dimensions (W x L x H): {self.default_width} x {self.default_length} x {self.default_height}")
        rospy.loginfo(f"- Shape: Box (cube)")
        rospy.loginfo(f"- Update rate: {self.update_rate} seconds")
        rospy.loginfo(f"- Obstacles will be saved to: {self.obstacles_csv_path}")
        rospy.loginfo("Waiting for 2D Nav Goals...")

    def goal_callback(self, pose_msg):
        """Callback function when 2D Nav Goal is selected"""
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        
        # Convert Quaternion to Euler angles
        quaternion = (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w
        )
        theta = self.quaternion_to_yaw(quaternion)
        
        # Add obstacle
        self.add_obstacle(x, y, theta)
        
    def add_obstacle(self, x, y, theta):
        """Add obstacle and save to CSV file"""
        # Generate obstacle ID (based on current time)
        obstacle_id = len(self.obstacles) + 1
        
        # New obstacle information
        obstacle = {
            'id': obstacle_id,
            'x': x,
            'y': y,
            'theta': theta,
            'width': self.default_width,
            'length': self.default_length,
            'height': self.default_height
        }
        
        self.obstacles.append(obstacle)
        self.save_obstacles()
        rospy.loginfo(f"Added new obstacle #{obstacle_id} at position: ({x:.2f}, {y:.2f}, {theta:.2f})")
        
    def save_obstacles(self):
        """Save obstacle information to CSV file"""        
        df = pd.DataFrame(self.obstacles)
        df.to_csv(self.obstacles_csv_path, index=False)
        rospy.loginfo(f"Saved {len(self.obstacles)} obstacles to {self.obstacles_csv_path}")
        
    def load_obstacles(self):
        """Load existing obstacle information"""
        if os.path.exists(self.obstacles_csv_path):
            df = pd.read_csv(self.obstacles_csv_path)
            self.obstacles = df.to_dict('records')
            rospy.loginfo(f"Loaded {len(self.obstacles)} obstacles from {self.obstacles_csv_path}")
        else:
            rospy.loginfo(f"No existing obstacles file found at {self.obstacles_csv_path}")
    
    def quaternion_to_yaw(self, quaternion):
        """Convert Quaternion to yaw angle (theta)"""
        x, y, z, w = quaternion
        
        # Calculate roll, pitch, yaw
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        
        return yaw
        
    def publish_markers(self, event=None):
        """Publish obstacle markers to RViz"""
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(self.obstacles):
            marker = self.create_box_marker(obstacle, i)
            marker_array.markers.append(marker)
            
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
        
        # Set size - modified part
        # In RViz CUBE marker, x is forward direction, y is left-right direction
        # Therefore, when moving in theta direction, length maps to x-axis, width maps to y-axis
        marker.scale.x = obstacle['length']  # Length (forward direction) - mapped to x-axis
        marker.scale.y = obstacle['width']   # Width (left-right direction) - mapped to y-axis
        marker.scale.z = obstacle['height']  # Height
        
        # Set color
        marker.color.r = self.marker_color['r']
        marker.color.g = self.marker_color['g']
        marker.color.b = self.marker_color['b']
        marker.color.a = self.marker_color['a']
        
        return marker

    def run(self):
        """Run node"""
        rospy.loginfo("Obstacle Manager is running. Press Ctrl+C to terminate.")
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_manager = ObstacleManager()
        obstacle_manager.run()
    except rospy.ROSInterruptException:
        pass