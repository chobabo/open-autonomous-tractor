#!/usr/bin/env python3
import rospy
import yaml
import csv
import os
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class FieldBoundaryVisualizer:
    def __init__(self):
        rospy.init_node('field_boundary_visualizer', anonymous=True)
        
        # Get requested map name
        requested_map = rospy.get_param('/requested_map', '')
        
        # Get package path
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('open_autonomous_tractor')
        
        # Configuration file path
        config_file = rospy.get_param('/config_file_path', 
                                     os.path.join(self.package_path, 'config', 'localization', 'saved_origins.yaml'))
        
        # Read saved_origins.yaml file
        with open(config_file, 'r') as f:
            self.origins_data = yaml.safe_load(f)
        
        # Use default value if map name is not specified
        if not requested_map:
            self.current_map = self.origins_data['default_origin']
        else:
            self.current_map = requested_map
        
        # Get current map's origin information
        if self.current_map in self.origins_data['saved_origins']:
            self.origin_info = self.origins_data['saved_origins'][self.current_map]
        else:
            rospy.logwarn(f"Map '{self.current_map}' not found in saved_origins.yaml. Using default.")
            default_map = self.origins_data['default_origin']
            self.origin_info = self.origins_data['saved_origins'][default_map]
            self.current_map = default_map
        
        # Set current map name in parameter server
        rospy.set_param('/current_map', self.current_map)
        
        # Set dataset path
        self.dataset_path = os.path.join(self.package_path, 'datasets', self.current_map)
        
        # Set up MarkerArray publisher
        self.marker_pub = rospy.Publisher('/field_boundaries', MarkerArray, queue_size=10)
        
        # Load field boundary data and create markers
        self.field_markers = self.create_field_markers()
        
        # Set up timer (periodically publish markers)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)
        
        rospy.loginfo(f"Field Boundary Visualizer initialized for map: {self.current_map}")
    
    def load_field_boundaries(self):
        """Load field boundary point data from CSV file"""
        fields_data = []
        boundaries_file = os.path.join(self.dataset_path, 'field_boundaries.csv')
        
        if not os.path.exists(boundaries_file):
            rospy.logerr(f"Field boundaries file not found: {boundaries_file}")
            return fields_data
        
        with open(boundaries_file, 'r') as f:
            csv_reader = csv.reader(f)
            for row in csv_reader:
                field_name = row[0]
                points = []
                # CSV format: field_name, P1y, P1x, P1z, P2y, P2x, P2z, ...
                for i in range(1, len(row), 3):
                    if i+2 < len(row):
                        # CSV stores coordinates in (origin_x, origin_y, origin_z) order
                        x = float(row[i])
                        y = float(row[i+1])
                        z = float(row[i+2])
                        points.append((x, y, z))
                
                fields_data.append({
                    'name': field_name,
                    'points': points
                })
        
        return fields_data
    
    def create_field_markers(self):
        """Create markers for field boundaries"""
        marker_array = MarkerArray()
        field_data = self.load_field_boundaries()
        
        for i, field in enumerate(field_data):
            # Add orientation to field boundary marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "field_boundaries"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.5  # Line thickness

            # Set quaternion (unit quaternion = no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Use different colors for each field
            color = ColorRGBA()
            color.r = 0.0 + (i * 0.3) % 1.0
            color.g = 0.5 + (i * 0.2) % 0.5
            color.b = 1.0 - (i * 0.3) % 1.0
            color.a = 1.0
            marker.color = color
            
            # Add boundary points
            for x, y, z in field['points']:
                point = Point()
                # Calculate relative position based on origin
                point.x = x - self.origin_info['origin_x']
                point.y = y - self.origin_info['origin_y']
                point.z = z - self.origin_info['origin_z']
                marker.points.append(point)
            
            # Add first point again to create a closed loop
            if field['points']:
                x, y, z = field['points'][0]
                point = Point()
                # Calculate relative position based on origin
                point.x = x - self.origin_info['origin_x']
                point.y = y - self.origin_info['origin_y']
                point.z = z - self.origin_info['origin_z']
                marker.points.append(point)
            
            marker_array.markers.append(marker)
            
            # Field name text marker
            if field['points']:
                # Calculate field center point
                center_x = sum(p[0] for p in field['points']) / len(field['points'])
                center_y = sum(p[1] for p in field['points']) / len(field['points'])
                center_z = sum(p[2] for p in field['points']) / len(field['points'])
                
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.ns = "field_names"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD

                # Add orientation to text marker as well
                text_marker.pose.orientation.x = 0.0
                text_marker.pose.orientation.y = 0.0
                text_marker.pose.orientation.z = 0.0
                text_marker.pose.orientation.w = 1.0                
                
                # Calculate relative position based on origin
                text_marker.pose.position.x = center_x - self.origin_info['origin_x']
                text_marker.pose.position.y = center_y - self.origin_info['origin_y']
                text_marker.pose.position.z = center_z - self.origin_info['origin_z'] + 1.0  # Display slightly above
                
                text_marker.scale.z = 2.0  # Text size
                text_marker.color = color
                text_marker.text = field['name']
                
                marker_array.markers.append(text_marker)
        
        return marker_array
    
    def publish_markers(self, event=None):
        """Publish markers"""
        if self.field_markers.markers:
            # Update timestamps
            for marker in self.field_markers.markers:
                marker.header.stamp = rospy.Time.now()
            
            self.marker_pub.publish(self.field_markers)
    
    def run(self):
        """Main execution function"""
        rospy.spin()

if __name__ == '__main__':
    try:
        visualizer = FieldBoundaryVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass