#!/usr/bin/env python3

import os
import csv
import math
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String


class GlobalPathGenerator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('global_path_generator', anonymous=True)
        
        # Get parameters from ROS parameter server
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.csv_file = rospy.get_param('~csv_file', 'recorded_poses.csv')
        self.output_csv_file = rospy.get_param('~output_csv_file', 'global_path.csv')  # Output CSV file
        self.auto_save = rospy.get_param('~auto_save', False)  # Automatically save to CSV
        self.path_spacing = rospy.get_param('~path_spacing', 0.1)  # Default spacing: 0.1m
        self.theta_threshold = rospy.get_param('~theta_threshold', 0.2)  # Default threshold: 0.2 rad
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Default rate: 1 Hz
        
        # Ensure CSV file exists
        if not os.path.exists(self.csv_file):
            rospy.logerr(f"CSV file not found: {self.csv_file}")
            exit(1)
        
        # Publishers
        self.path_pub = rospy.Publisher('global_path', Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('path_markers', MarkerArray, queue_size=1, latch=True)
        self.poses_pub = rospy.Publisher('original_poses', MarkerArray, queue_size=1, latch=True)
        
        # Services for saving to CSV
        self.save_service = rospy.Service('save_global_path', Trigger, self.save_path_callback)
        
        # Create a subscriber for custom path saving
        self.save_path_sub = rospy.Subscriber('save_path_to_file', String, self.save_path_topic_callback)
        
        # Load poses from CSV file
        self.poses = self.load_poses_from_csv(self.csv_file)
        
        if not self.poses:
            rospy.logerr("No poses loaded from CSV file")
            exit(1)
            
        rospy.loginfo(f"Loaded {len(self.poses)} poses from {self.csv_file}")
        
        # Generate global path
        self.global_path = self.generate_global_path(self.poses, self.path_spacing, self.theta_threshold)
        
        rospy.loginfo(f"Generated global path with {len(self.global_path)} points")
        
        # Setup timer for publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_callback)
        
        # Save path to CSV if auto_save is enabled
        if self.auto_save:
            self.save_path_to_csv(self.output_csv_file)
            rospy.loginfo(f"Automatically saved global path to {self.output_csv_file}")
        
    def load_poses_from_csv(self, csv_file):
        """Load poses from CSV file with format: id, x, y, theta"""
        poses = []
        
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    pose = {
                        'id': int(row['id']),
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'theta': float(row['theta'])
                    }
                    poses.append(pose)
        except Exception as e:
            rospy.logerr(f"Error loading CSV file: {e}")
            return []
            
        return poses
    
    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def interpolate_angle(self, start_angle, end_angle, ratio):
        """Interpolate between two angles, taking the shortest path"""
        # Normalize angles
        start_angle = self.normalize_angle(start_angle)
        end_angle = self.normalize_angle(end_angle)
        
        # Find the shortest path
        diff = self.normalize_angle(end_angle - start_angle)
        
        # Apply interpolation
        interpolated = start_angle + diff * ratio
        
        # Return normalized result
        return self.normalize_angle(interpolated)
        
    def distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
    
    def generate_global_path(self, poses, spacing=0.1, theta_threshold=0.2):
        """Generate a global path with configurable spacing"""
        if len(poses) < 2:
            return poses
        
        global_path = []
        
        # Add the first pose to the path
        global_path.append({
            'x': poses[0]['x'],
            'y': poses[0]['y'],
            'theta': poses[0]['theta']
        })
        
        # Process consecutive poses
        for i in range(1, len(poses)):
            start_pose = poses[i-1]
            end_pose = poses[i]
            
            # Calculate the distance between current pair of poses
            dist = self.distance(start_pose, end_pose)
            
            # Calculate theta difference - ensure it's properly normalized
            theta_diff = abs(self.normalize_angle(end_pose['theta'] - start_pose['theta']))
            
            # Determine if we should use a straight line or a curve
            if theta_diff <= theta_threshold:
                # Use a straight line interpolation for similar theta values
                num_points = max(1, int(dist / spacing))
                
                for j in range(1, num_points + 1):
                    ratio = j / num_points
                    
                    # Linear interpolation
                    x = start_pose['x'] + ratio * (end_pose['x'] - start_pose['x'])
                    y = start_pose['y'] + ratio * (end_pose['y'] - start_pose['y'])
                    
                    # Linear interpolation of angle - ensure proper wraparound
                    theta = self.interpolate_angle(start_pose['theta'], end_pose['theta'], ratio)
                    
                    # Add the interpolated point to the path
                    global_path.append({'x': x, 'y': y, 'theta': theta})
            else:
                # Use a curved path (cubic Bezier) for significant theta changes
                num_points = max(1, int(dist / spacing))
                
                # Calculate control points for the Bezier curve
                # These affect the shape of the curve based on theta
                ctrl_dist = dist / 3
                
                # Control point 1: extend from start point in direction of start theta
                ctrl1 = {
                    'x': start_pose['x'] + ctrl_dist * math.cos(start_pose['theta']),
                    'y': start_pose['y'] + ctrl_dist * math.sin(start_pose['theta'])
                }
                
                # Control point 2: extend from end point in opposite direction of end theta
                ctrl2 = {
                    'x': end_pose['x'] - ctrl_dist * math.cos(end_pose['theta']),
                    'y': end_pose['y'] - ctrl_dist * math.sin(end_pose['theta'])
                }
                
                # Generate points along the Bezier curve
                for j in range(1, num_points + 1):
                    t = j / num_points
                    
                    # Cubic Bezier formula
                    x = (1-t)**3 * start_pose['x'] + \
                        3 * (1-t)**2 * t * ctrl1['x'] + \
                        3 * (1-t) * t**2 * ctrl2['x'] + \
                        t**3 * end_pose['x']
                    
                    y = (1-t)**3 * start_pose['y'] + \
                        3 * (1-t)**2 * t * ctrl1['y'] + \
                        3 * (1-t) * t**2 * ctrl2['y'] + \
                        t**3 * end_pose['y']
                    
                    # For theta, use linear interpolation with proper angle handling
                    theta = self.interpolate_angle(start_pose['theta'], end_pose['theta'], t)
                    
                    # Add the interpolated point to the path
                    global_path.append({'x': x, 'y': y, 'theta': theta})
        
        return global_path
    
    def create_path_msg(self, path_points):
        """Create a nav_msgs/Path message from the path points"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id
        
        for point in path_points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.frame_id
            
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = 0.0
            
            # Convert theta to quaternion - ensure normalized quaternion
            theta = point['theta']
            # We're rotating around the Z axis
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            # Normalize quaternion to ensure unit length
            norm = math.sqrt(qz * qz + qw * qw)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz / norm
            pose.pose.orientation.w = qw / norm
            
            path_msg.poses.append(pose)
        
        return path_msg
    
    def create_marker_array(self, path_points, marker_type='lines'):
        """Create visualization markers for the path"""
        marker_array = MarkerArray()
        
        if marker_type == 'lines':
            # Create a line strip marker for the path
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "global_path"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Line width
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
            
            for point in path_points:
                p = Point()
                p.x = point['x']
                p.y = point['y']
                p.z = 0.1  # Slightly above ground for visibility
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        elif marker_type == 'arrows':
            # Create arrow markers for the original poses
            for i, point in enumerate(path_points):
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = "original_poses"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                marker.pose.position.x = point['x']
                marker.pose.position.y = point['y']
                marker.pose.position.z = 0.2  # Slightly above ground
                
                # Convert theta to quaternion - ensure normalized quaternion
                theta = point['theta']
                # We're rotating around the Z axis
                qz = math.sin(theta / 2.0)
                qw = math.cos(theta / 2.0)
                # Normalize quaternion to ensure unit length
                norm = math.sqrt(qz * qz + qw * qw)
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = qz / norm
                marker.pose.orientation.w = qw / norm
                
                # Arrow size
                marker.scale.x = 0.3  # Arrow length
                marker.scale.y = 0.05  # Arrow width
                marker.scale.z = 0.05  # Arrow height
                
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
                
                marker_array.markers.append(marker)
        
        return marker_array
    
    def save_path_to_csv(self, filename):
        """Save the global path to a CSV file"""
        try:
            # Create directory if it doesn't exist
            directory = os.path.dirname(filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
                
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                # Write header
                writer.writerow(['id', 'x', 'y', 'theta'])
                
                # Write path points
                for i, point in enumerate(self.global_path):
                    writer.writerow([i+1, point['x'], point['y'], point['theta']])
                    
            rospy.loginfo(f"Global path saved to {filename}")
            return True
        except Exception as e:
            rospy.logerr(f"Error saving global path to CSV: {e}")
            return False
    
    def save_path_callback(self, req):
        """Service callback to save the global path to CSV using Trigger service"""
        # Use default filename
        filename = self.output_csv_file
        
        if self.save_path_to_csv(filename):
            return TriggerResponse(success=True, message=f"Global path saved to {filename}")
        else:
            return TriggerResponse(success=False, message=f"Failed to save global path to {filename}")
    
    def save_path_topic_callback(self, msg):
        """Topic callback to save the global path to a custom filename"""
        # If message data is empty, use default filename
        filename = msg.data if msg.data else self.output_csv_file
        
        if self.save_path_to_csv(filename):
            rospy.loginfo(f"Global path saved to {filename}")
        else:
            rospy.logerr(f"Failed to save global path to {filename}")
            
    def publish_callback(self, event):
        """Publish the global path and visualization markers"""
        # Create and publish Path message
        path_msg = self.create_path_msg(self.global_path)
        self.path_pub.publish(path_msg)
        
        # Create and publish marker arrays
        path_markers = self.create_marker_array(self.global_path, 'lines')
        self.marker_pub.publish(path_markers)
        
        # Create and publish original pose markers
        pose_markers = self.create_marker_array(self.poses, 'arrows')
        self.poses_pub.publish(pose_markers)
        
        rospy.logdebug("Published global path and markers")

if __name__ == '__main__':
    try:
        generator = GlobalPathGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass