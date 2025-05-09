#!/usr/bin/env python3
import rospy
import yaml
import csv
import os
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class RoadBoundaryVisualizer:
    def __init__(self):
        rospy.init_node('road_boundary_visualizer', anonymous=True)
        
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
        self.marker_pub = rospy.Publisher('/road_boundaries', MarkerArray, queue_size=10)
        
        # Load road boundary data and create markers
        self.road_markers = self.create_road_markers()
        
        # Set up timer (periodically publish markers)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)
        
        rospy.loginfo(f"Road Boundary Visualizer initialized for map: {self.current_map}")
    
    def load_road_boundary_points(self):
        """Load road boundary point data from CSV file"""
        points_dict = {}
        points_file = os.path.join(self.dataset_path, 'road_boundary_points.csv')
        
        if not os.path.exists(points_file):
            rospy.logerr(f"Road boundary points file not found: {points_file}")
            return points_dict
        
        with open(points_file, 'r') as f:
            csv_reader = csv.reader(f)
            next(csv_reader, None)  # Skip header (id, latitude, longitude, y, x)
            for row in csv_reader:
                if len(row) >= 5:
                    point_id = int(row[0])
                    # CSV format: id, latitude, longitude, y, x
                    # y and x are in plane rectangular coordinate system (origin_y, origin_x)
                    y = float(row[3])
                    x = float(row[4])
                    z = 0.0  # Set height to 0 if not available
                    
                    points_dict[point_id] = (x, y, z)
        
        return points_dict
    
    def load_road_boundary_links(self):
        """Load road boundary link data from CSV file"""
        links = []
        links_file = os.path.join(self.dataset_path, 'road_boundary_links.csv')
        
        if not os.path.exists(links_file):
            rospy.logerr(f"Road boundary links file not found: {links_file}")
            return links
        
        with open(links_file, 'r') as f:
            csv_reader = csv.reader(f)
            next(csv_reader, None)  # Skip header (start_id, end_id)
            for row in csv_reader:
                if len(row) >= 2:
                    start_id = int(row[0])
                    end_id = int(row[1])
                    links.append((start_id, end_id))
        
        return links
    
    def create_road_markers(self):
        """Create markers for road boundaries"""
        marker_array = MarkerArray()
        road_points = self.load_road_boundary_points()
        road_links = self.load_road_boundary_links()
        
        if not road_points or not road_links:
            rospy.logwarn("No road boundary data found. Markers not created.")
            return marker_array
        
        # Create road boundary marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "road_boundaries"
        marker.id = 0
        marker.type = Marker.LINE_LIST  # Display as individual line segments
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # Line thickness
        marker.pose.orientation.w = 1.0
        
        # Set road boundary color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Connect start and end points for each link
        for start_id, end_id in road_links:
            if start_id in road_points and end_id in road_points:
                # Start point
                start_x, start_y, start_z = road_points[start_id]
                start_point = Point()
                # Calculate relative position based on origin
                start_point.x = start_x - self.origin_info['origin_x']
                start_point.y = start_y - self.origin_info['origin_y']
                start_point.z = start_z - self.origin_info['origin_z']
                marker.points.append(start_point)
                
                # End point
                end_x, end_y, end_z = road_points[end_id]
                end_point = Point()
                # Calculate relative position based on origin
                end_point.x = end_x - self.origin_info['origin_x']
                end_point.y = end_y - self.origin_info['origin_y']
                end_point.z = end_z - self.origin_info['origin_z']
                marker.points.append(end_point)
        
        marker_array.markers.append(marker)
        
        # Create road boundary point marker (display as small spheres)
        point_marker = Marker()
        point_marker.header.frame_id = "map"
        point_marker.ns = "road_boundary_points"
        point_marker.id = 1
        point_marker.type = Marker.SPHERE_LIST
        point_marker.action = Marker.ADD
        point_marker.scale.x = 0.5  # Sphere size
        point_marker.scale.y = 0.5
        point_marker.scale.z = 0.5
        point_marker.pose.orientation.w = 1.0
        
        # Set road boundary point color (blue)
        point_marker.color.r = 0.0
        point_marker.color.g = 0.0
        point_marker.color.b = 1.0
        point_marker.color.a = 1.0
        
        # Add each boundary point
        for point_id, (x, y, z) in road_points.items():
            point = Point()
            # Calculate relative position based on origin
            point.x = x - self.origin_info['origin_x']
            point.y = y - self.origin_info['origin_y']
            point.z = z - self.origin_info['origin_z']
            point_marker.points.append(point)
        
        marker_array.markers.append(point_marker)
        
        return marker_array
    
    def publish_markers(self, event=None):
        """Publish markers"""
        if self.road_markers.markers:
            # Update timestamps
            for marker in self.road_markers.markers:
                marker.header.stamp = rospy.Time.now()
            
            self.marker_pub.publish(self.road_markers)
    
    def run(self):
        """Main execution function"""
        rospy.spin()

if __name__ == '__main__':
    try:
        visualizer = RoadBoundaryVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass