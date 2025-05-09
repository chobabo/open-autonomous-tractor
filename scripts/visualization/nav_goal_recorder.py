#!/usr/bin/env python3

import rospy
import csv
import os
import math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class NavGoalRecorder:
    def __init__(self):
        rospy.init_node('nav_goal_recorder', anonymous=True)
        
        # Parameters
        self.save_dir = rospy.get_param('~save_dir', os.path.expanduser('~') + '/catkin_ws/src/open_autonomous_tractor/datasets/recorded_poses')
        self.csv_filename = rospy.get_param('~csv_filename', 'recorded_poses.csv')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # Full path to CSV file
        self.csv_path = os.path.join(self.save_dir, self.csv_filename)
        
        # List to store poses
        self.poses = []
        self.next_id = 0
        
        # Load existing poses if file exists
        if os.path.exists(self.csv_path):
            with open(self.csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # Skip header
                for row in reader:
                    if len(row) == 4:
                        id_val = int(row[0])
                        x = float(row[1])
                        y = float(row[2])
                        theta = float(row[3])
                        self.poses.append({'id': id_val, 'x': x, 'y': y, 'theta': theta})
                        if id_val >= self.next_id:
                            self.next_id = id_val + 1
        else:
            # Create new file with header
            with open(self.csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['id', 'x', 'y', 'theta'])
        
        # Create subscribers and publishers
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.marker_pub = rospy.Publisher('/recorded_poses/markers', MarkerArray, queue_size=10)
        
        # Timer for visualization updates
        self.vis_timer = rospy.Timer(rospy.Duration(0.1), self.visualize_poses)
        
        rospy.loginfo(f"Nav Goal Recorder started. Poses will be saved to {self.csv_path}")
        rospy.loginfo("Click on 2D Nav Goal in RViz to record poses.")
    
    def goal_callback(self, goal_msg):
        """Callback for 2D Nav Goal"""
        x = goal_msg.pose.position.x
        y = goal_msg.pose.position.y
        
        # Extract theta (yaw) from quaternion
        qx = goal_msg.pose.orientation.x
        qy = goal_msg.pose.orientation.y
        qz = goal_msg.pose.orientation.z
        qw = goal_msg.pose.orientation.w
        
        # Convert quaternion to Euler angles
        t3 = 2.0 * (qw * qz + qx * qy)
        t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = math.atan2(t3, t4)
        
        # Create pose record
        pose = {'id': self.next_id, 'x': x, 'y': y, 'theta': theta}
        self.poses.append(pose)
        
        # Save to CSV
        with open(self.csv_path, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([pose['id'], pose['x'], pose['y'], pose['theta']])
        
        rospy.loginfo(f"Recorded pose {self.next_id}: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
        self.next_id += 1
    
    def visualize_poses(self, event=None):
        """Visualize recorded poses with text and arrows"""
        marker_array = MarkerArray()
        
        for i, pose in enumerate(self.poses):
            # Text marker for pose info
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "pose_texts"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose['x']
            text_marker.pose.position.y = pose['y']
            text_marker.pose.position.z = 0.5  # Offset above ground
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # Text size
            text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White
            text_marker.text = f"ID: {pose['id']}\nx: {pose['x']:.2f}\ny: {pose['y']:.2f}\nθ: {pose['theta']:.2f}"
            marker_array.markers.append(text_marker)
            
            # Arrow marker for direction
            arrow_marker = Marker()
            arrow_marker.header.frame_id = self.frame_id
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "pose_arrows"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = pose['x']
            arrow_marker.pose.position.y = pose['y']
            arrow_marker.pose.position.z = 0.1
            
            # Set arrow orientation from theta
            arrow_marker.pose.orientation.x = 0.0
            arrow_marker.pose.orientation.y = 0.0
            arrow_marker.pose.orientation.z = math.sin(pose['theta'] / 2.0)
            arrow_marker.pose.orientation.w = math.cos(pose['theta'] / 2.0)
            
            # Arrow size
            arrow_marker.scale.x = 0.8  # Arrow length
            arrow_marker.scale.y = 0.1  # Arrow width
            arrow_marker.scale.z = 0.1  # Arrow height
            arrow_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
            marker_array.markers.append(arrow_marker)
        
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        recorder = NavGoalRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass