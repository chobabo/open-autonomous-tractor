#!/usr/bin/env python3

import os
import csv
import math
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header, Float64MultiArray
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
        
        # Ensure input CSV file exists
        if not os.path.exists(self.csv_file):
            rospy.logerr(f"Input CSV file not found: {self.csv_file}. Please ensure recorded_poses.csv exists.")
            exit(1)
        
        # Publishers
        self.path_pub = rospy.Publisher('global_path', Path, queue_size=1, latch=True)
        self.additional_path_info_pub = rospy.Publisher('global_path_bounds', Float64MultiArray, queue_size=1, latch=True) # 추가 정보 발행
        self.marker_pub = rospy.Publisher('path_markers', MarkerArray, queue_size=1, latch=True)
        self.poses_pub = rospy.Publisher('original_poses', MarkerArray, queue_size=1, latch=True)
        
        # Services for saving to CSV
        self.save_service = rospy.Service('save_global_path', Trigger, self.save_path_callback)
        
        # Create a subscriber for custom path saving
        self.save_path_sub = rospy.Subscriber('save_path_to_file', String, self.save_path_topic_callback)
        
        # Load poses from CSV file (now includes bounds)
        self.recorded_poses = self.load_poses_from_csv(self.csv_file)
        
        if not self.recorded_poses:
            rospy.logerr("No poses loaded from input CSV file. Exiting.")
            exit(1)
            
        rospy.loginfo(f"Loaded {len(self.recorded_poses)} poses from {self.csv_file}")
        
        # Generate global path (now includes s, left_bound_d, right_bound_d)
        self.global_path = self.generate_global_path(self.recorded_poses, self.path_spacing, self.theta_threshold)
        
        rospy.loginfo(f"Generated global path with {len(self.global_path)} points")
        
        # Setup timer for publishing
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_callback)
        
        # Save path to CSV if auto_save is enabled
        if self.auto_save:
            self.save_path_to_csv(self.output_csv_file)
            rospy.loginfo(f"Automatically saved global path to {self.output_csv_file}")
            
    def load_poses_from_csv(self, csv_file):
        """
        Load poses from CSV file.
        Expected format: id, x, y, theta, left_bound_d, right_bound_d
        """
        poses = []
        try:
            with open(csv_file, 'r', newline='') as f:
                reader = csv.DictReader(f)
                # Ensure all required headers exist
                required_headers = ['id', 'x', 'y', 'theta', 'left_bound_d', 'right_bound_d']
                if not all(header in reader.fieldnames for header in required_headers):
                    rospy.logerr(f"CSV file '{csv_file}' is missing required headers. Expected: {required_headers}, Found: {reader.fieldnames}")
                    return []

                for row in reader:
                    try:
                        pose = {
                            'id': int(row['id']),
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'theta': float(row['theta']),
                            'left_bound_d': float(row['left_bound_d']),
                            'right_bound_d': float(row['right_bound_d'])
                        }
                        poses.append(pose)
                    except ValueError as e:
                        rospy.logwarn(f"Skipping row due to data conversion error in {csv_file}: {row} - {e}")
        except Exception as e:
            rospy.logerr(f"Error loading CSV file '{csv_file}': {e}")
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
        """Calculate Euclidean distance between two points (dictionary with x, y)"""
        return math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)
    
    def generate_global_path(self, recorded_poses, spacing=0.1, theta_threshold=0.2):
        """
        Generate a global path with configurable spacing,
        including s, left_bound_d, and right_bound_d.
        The bounds are taken from the start waypoint of each segment.
        """
        if len(recorded_poses) < 2:
            # If less than 2 poses, just return the first pose with s=0
            if recorded_poses:
                return [{
                    'x': recorded_poses[0]['x'],
                    'y': recorded_poses[0]['y'],
                    'theta': recorded_poses[0]['theta'],
                    's': 0.0,
                    'left_bound_d': recorded_poses[0]['left_bound_d'],
                    'right_bound_d': recorded_poses[0]['right_bound_d']
                }]
            return []
        
        global_path = []
        current_s = 0.0 # Initialize cumulative path length
        
        # Add the first pose to the path with s=0
        global_path.append({
            'x': recorded_poses[0]['x'],
            'y': recorded_poses[0]['y'],
            'theta': recorded_poses[0]['theta'],
            's': current_s,
            'left_bound_d': recorded_poses[0]['left_bound_d'],
            'right_bound_d': recorded_poses[0]['right_bound_d']
        })
        
        # Process consecutive poses
        for i in range(1, len(recorded_poses)):
            start_pose = recorded_poses[i-1]
            end_pose = recorded_poses[i]
            
            # The bounds for the entire segment will be from the start_pose
            segment_left_bound_d = start_pose['left_bound_d']
            segment_right_bound_d = start_pose['right_bound_d']

            # Calculate the distance between current pair of poses
            dist = self.distance(start_pose, end_pose)
            
            # Calculate theta difference - ensure it's properly normalized
            theta_diff = abs(self.normalize_angle(end_pose['theta'] - start_pose['theta']))
            
            num_points_in_segment = max(1, int(dist / spacing))
            
            # Use a curved path (cubic Bezier) or straight line
            if theta_diff <= theta_threshold:
                # Straight line interpolation for similar theta values
                for j in range(1, num_points_in_segment + 1):
                    ratio = j / num_points_in_segment
                    
                    x = start_pose['x'] + ratio * (end_pose['x'] - start_pose['x'])
                    y = start_pose['y'] + ratio * (end_pose['y'] - start_pose['y'])
                    theta = self.interpolate_angle(start_pose['theta'], end_pose['theta'], ratio)
                    
                    # Update current_s
                    prev_x = global_path[-1]['x']
                    prev_y = global_path[-1]['y']
                    current_s += math.hypot(x - prev_x, y - prev_y)

                    global_path.append({
                        'x': x,
                        'y': y,
                        'theta': theta,
                        's': current_s,
                        'left_bound_d': segment_left_bound_d, # Apply start waypoint's bound
                        'right_bound_d': segment_right_bound_d # Apply start waypoint's bound
                    })
            else:
                # Use a curved path (cubic Bezier) for significant theta changes
                ctrl_dist = dist / 3 # Control point distance
                
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
                for j in range(1, num_points_in_segment + 1):
                    t = j / num_points_in_segment
                    
                    # Cubic Bezier formula for x, y
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
                    
                    # Update current_s
                    prev_x = global_path[-1]['x']
                    prev_y = global_path[-1]['y']
                    current_s += math.hypot(x - prev_x, y - prev_y)
                    
                    global_path.append({
                        'x': x,
                        'y': y,
                        'theta': theta,
                        's': current_s,
                        'left_bound_d': segment_left_bound_d, # Apply start waypoint's bound
                        'right_bound_d': segment_right_bound_d # Apply start waypoint's bound
                    })
        
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
            pose.pose.position.z = 0.0 # Z is typically 0 for 2D paths
            
            # Convert theta to quaternion - ensure normalized quaternion
            theta = point['theta']
            # We're rotating around the Z axis
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            # Normalize quaternion to ensure unit length (avoiding numerical issues)
            norm = math.sqrt(qz * qz + qw * qw)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz / norm if norm != 0 else 0.0
            pose.pose.orientation.w = qw / norm if norm != 0 else 1.0
            
            path_msg.poses.append(pose)
        
        return path_msg

    def create_additional_path_info_msg(self, path_points):
        """Create a Float64MultiArray message for s, left_bound_d, right_bound_d"""
        info_msg = Float64MultiArray()
        # Layout: [s1, left_d1, right_d1, s2, left_d2, right_d2, ...]
        info_msg.data = []
        for point in path_points:
            info_msg.data.append(point['s'])
            info_msg.data.append(point['left_bound_d'])
            info_msg.data.append(point['right_bound_d'])
        return info_msg

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

            # Add markers for left/right boundaries along the path
            # This creates line segments perpendicular to the path at each point
            for i, point in enumerate(path_points):
                # Only add boundary markers for some points to avoid clutter, e.g., every 5th point
                if i % 10 == 0 or i == 0 or i == len(path_points) - 1: # Add at start, end, and every 10 points
                    # Calculate points for the boundary line segment
                    # Perpendicular direction to the pose's heading (pointing left)
                    perp_x = -math.sin(point['theta'])
                    perp_y = math.cos(point['theta'])

                    left_point_vis = Point()
                    left_point_vis.x = point['x'] + point.get('left_bound_d', 0.0) * perp_x
                    left_point_vis.y = point['y'] + point.get('left_bound_d', 0.0) * perp_y
                    left_point_vis.z = 0.05 # Slightly above path line

                    right_point_vis = Point()
                    right_point_vis.x = point['x'] - point.get('right_bound_d', 0.0) * perp_x
                    right_point_vis.y = point['y'] - point.get('right_bound_d', 0.0) * perp_y
                    right_point_vis.z = 0.05
                    
                    boundary_line_marker = Marker()
                    boundary_line_marker.header.frame_id = self.frame_id
                    boundary_line_marker.header.stamp = rospy.Time.now()
                    boundary_line_marker.ns = "global_path_boundaries"
                    boundary_line_marker.id = i # Unique ID for each boundary line
                    boundary_line_marker.type = Marker.LINE_LIST
                    boundary_line_marker.action = Marker.ADD
                    boundary_line_marker.scale.x = 0.03 # Line width for boundaries
                    boundary_line_marker.color = ColorRGBA(1.0, 0.5, 0.0, 0.7) # Orange transparent
                    boundary_line_marker.points.append(left_point_vis)
                    boundary_line_marker.points.append(right_point_vis)
                    
                    marker_array.markers.append(boundary_line_marker)
        
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
                qz = math.sin(theta / 2.0)
                qw = math.cos(theta / 2.0)
                norm = math.sqrt(qz * qz + qw * qw)
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = qz / norm if norm != 0 else 0.0
                marker.pose.orientation.w = qw / norm if norm != 0 else 1.0
                
                # Arrow size
                marker.scale.x = 0.3  # Arrow length
                marker.scale.y = 0.05  # Arrow width
                marker.scale.z = 0.05  # Arrow height
                
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
                
                marker_array.markers.append(marker)
        
        return marker_array
    
    def save_path_to_csv(self, filename):
        """Save the global path to a CSV file, including s and bounds."""
        try:
            directory = os.path.dirname(filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
                
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                # Write header including new fields
                writer.writerow(['id', 'x', 'y', 'theta', 's', 'left_bound_d', 'right_bound_d'])
                
                # Write path points
                for i, point in enumerate(self.global_path):
                    writer.writerow([
                        i, # Use i as id for interpolated points
                        point['x'],
                        point['y'],
                        point['theta'],
                        point['s'],
                        point['left_bound_d'],
                        point['right_bound_d']
                    ])
                    
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
        filename = msg.data if msg.data else self.output_csv_file
        
        if self.save_path_to_csv(filename):
            rospy.loginfo(f"Global path saved to {filename}")
        else:
            rospy.logerr(f"Failed to save global path to {filename}")
            
    def publish_callback(self, event):
        """Publish the global path and visualization markers"""
        # Create and publish nav_msgs/Path message
        path_msg = self.create_path_msg(self.global_path)
        self.path_pub.publish(path_msg)
        
        # Create and publish additional path info (s, left_bound_d, right_bound_d)
        additional_info_msg = self.create_additional_path_info_msg(self.global_path)
        self.additional_path_info_pub.publish(additional_info_msg)

        # Create and publish path line markers
        path_markers = self.create_marker_array(self.global_path, 'lines')
        self.marker_pub.publish(path_markers)
        
        # Create and publish original pose arrow markers
        pose_markers = self.create_marker_array(self.recorded_poses, 'arrows') # Use recorded_poses for original arrows
        self.poses_pub.publish(pose_markers)
        
        rospy.logdebug("Published global path, additional info, and markers")

if __name__ == '__main__':
    try:
        generator = GlobalPathGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass