#!/usr/bin/env python3

import rospy
import math
import yaml
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class TractorOdometry:
    def __init__(self):
        rospy.init_node('tractor_odometry_publisher')
        
        # Check tractor model type (default: tractor_model1)
        tractor_model = rospy.get_param('~tractor_model', 'tractor_model')
        
        # Get tractor parameters
        self.wheelbase = rospy.get_param(f'/{tractor_model}/wheelbase', 2.4)  # Wheelbase (m)
        self.max_steering_angle = rospy.get_param(f'/{tractor_model}/steering/max_angle', 0.785)  # Max steering angle (rad)
        
        rospy.loginfo(f"Loaded tractor model parameters: wheelbase={self.wheelbase}m, max_steering={self.max_steering_angle}rad")
        
        # Load map origin information for coordinate transformation
        origin_name = rospy.get_param('~origin_name', 'iam_field')
        origins_file = rospy.get_param('~origins_file', '')
        
        if not origins_file:
            # Default path - expand ROS package path
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('open_autonomous_tractor')
            origins_file = f"{pkg_path}/config/localization/saved_origins.yaml"
        
        # Load origin data
        self.origin_data = self.load_origin_from_yaml(origins_file, origin_name)
        if not self.origin_data:
            rospy.logerr("Failed to load map origin information")
            # Use default values if loading fails
            self.map_origin_x = 0.0
            self.map_origin_y = 0.0
        else:
            self.map_origin_x = self.origin_data['origin_x']
            self.map_origin_y = self.origin_data['origin_y']
        
        # Get initial pose from simulation parameters (absolute coordinates)
        initial_x_abs = rospy.get_param('/simulation/initial_pose/x', 0.0)
        initial_y_abs = rospy.get_param('/simulation/initial_pose/y', 0.0)
        initial_yaw = rospy.get_param('/simulation/initial_pose/yaw_rad', 0.0)
        
        # Other parameters
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Publishing rate (Hz)
        
        # Calculate MAP RELATIVE coordinates (same as path)
        # This is the key: both odometry and path use map-relative coordinates
        self.x = initial_x_abs - self.map_origin_x
        self.y = initial_y_abs - self.map_origin_y
        self.yaw = initial_yaw
        
        # Current velocity and steering angle
        self.velocity = 0.0  # Forward velocity (m/s)
        self.steering_angle = 0.0  # Steering angle (rad)
        
        # Last update time
        self.last_time = rospy.Time.now()
        
        # Debug mode
        self.debug = rospy.get_param('~debug', True)
        
        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/steering_angle', Float64, self.steering_callback)
        
        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Timer setup
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.update_odometry)
        
        rospy.loginfo("Tractor odometry node started")
        rospy.loginfo(f"Map origin: ({self.map_origin_x:.2f}, {self.map_origin_y:.2f})")
        rospy.loginfo(f"Initial position (absolute): ({initial_x_abs:.2f}, {initial_y_abs:.2f})")
        rospy.loginfo(f"Initial position (map-relative): ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})")
    
    def load_origin_from_yaml(self, file_path, origin_name):
        """Load map origin information from YAML file"""
        try:
            # Expand file path if it contains ROS package syntax
            if '$(find' in file_path:
                import rospkg
                rospack = rospkg.RosPack()
                # Simple replacement for $(find package) syntax
                file_path = file_path.replace('$(find frei_tractor)', rospack.get_path('frei_tractor'))
            
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                
            if 'saved_origins' not in data or origin_name not in data['saved_origins']:
                rospy.logerr(f"Origin '{origin_name}' not found in {file_path}")
                return None
                
            return data['saved_origins'][origin_name]
        except Exception as e:
            rospy.logerr(f"Failed to load origin info from {file_path}: {e}")
            return None
    
    def cmd_vel_callback(self, msg):
        """Velocity command callback"""
        # Extract forward velocity from Twist message
        self.velocity = msg.linear.x
        if self.debug:
            rospy.logdebug_throttle(2.0, f"Received velocity command: {self.velocity} m/s")
    
    def steering_callback(self, msg):
        """Steering angle callback"""
        self.steering_angle = msg.data
        if self.debug:
            rospy.logdebug_throttle(2.0, f"Received steering command: {self.steering_angle} rad ({math.degrees(self.steering_angle):.1f}°)")
    
    def update_odometry(self, event):
        """Periodically update odometry"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        if dt > 0 and abs(self.velocity) > 0.001:  # Update only when velocity is meaningful
            # Apply kinematic equations for car-like robot (Ackermann model)
            if abs(self.steering_angle) < 0.001:  # Almost straight motion
                # Straight motion
                delta_x = self.velocity * dt * math.cos(self.yaw)
                delta_y = self.velocity * dt * math.sin(self.yaw)
                delta_yaw = 0.0
                
                if self.debug:
                    rospy.logdebug_throttle(2.0, f"Straight motion: dx={delta_x:.3f}, dy={delta_y:.3f}, dyaw={delta_yaw:.3f}")
            else:
                # Turning motion
                # Calculate turning radius
                turning_radius = self.wheelbase / math.tan(abs(self.steering_angle))
                
                # Angular velocity about the turning center
                angular_velocity = self.velocity / turning_radius
                if self.steering_angle < 0:
                    angular_velocity = -angular_velocity
                
                # Calculate position and orientation changes
                delta_yaw = angular_velocity * dt
                
                # Calculate position change according to Ackermann model
                # Arc motion around the turning center
                if abs(delta_yaw) > 0.001:  # When orientation change is non-negligible
                    # Use arc motion formula
                    delta_x = (turning_radius * math.sin(self.yaw + delta_yaw) - 
                               turning_radius * math.sin(self.yaw))
                    delta_y = (turning_radius * math.cos(self.yaw) - 
                               turning_radius * math.cos(self.yaw + delta_yaw))
                    
                    # Sign adjustment (depending on steering direction)
                    if self.steering_angle < 0:
                        delta_x = -delta_x
                        delta_y = -delta_y
                else:
                    # Approximate small turns as straight line
                    delta_x = self.velocity * dt * math.cos(self.yaw)
                    delta_y = self.velocity * dt * math.sin(self.yaw)
                
                if self.debug:
                    rospy.logdebug_throttle(2.0, f"Turning motion: r={turning_radius:.3f}, ω={angular_velocity:.3f}, " +
                                 f"dx={delta_x:.3f}, dy={delta_y:.3f}, dyaw={delta_yaw:.3f}")
            
            # Update current position and orientation (map-relative coordinates)
            self.x += delta_x
            self.y += delta_y
            self.yaw += delta_yaw
            
            # Normalize yaw angle (-π ~ π)
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            
            if self.debug:
                rospy.logdebug_throttle(1.0, f"Updated position (map-relative): x={self.x:.3f}, y={self.y:.3f}, " +
                             f"yaw={self.yaw:.3f} rad ({math.degrees(self.yaw):.1f}°)")
            
            # Publish odometry message
            self.publish_odometry(current_time)
        elif dt > 0:
            # Publish odometry message with latest timestamp even when velocity is 0
            self.publish_odometry(current_time)
    
    def publish_odometry(self, timestamp):
        """Publish odometry message and TF"""
        # 1. Publish TF (odom → base_footprint)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        
        # Set position (map-relative coordinates)
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        
        # Set orientation (important: convert yaw value to rotation)
        q = quaternion_from_euler(0, 0, self.yaw)
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        
        # Publish TF
        self.tf_broadcaster.sendTransform(tf_msg)
        
        # 2. Publish odometry message
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Set position (map-relative coordinates)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocity
        odom.twist.twist.linear.x = self.velocity
        
        # Calculate angular velocity (Ackermann model)
        if abs(self.steering_angle) > 0.001 and abs(self.velocity) > 0.001:
            # Calculate angular velocity based on turning radius
            angular_velocity = self.velocity * math.tan(self.steering_angle) / self.wheelbase
            odom.twist.twist.angular.z = angular_velocity
        else:
            odom.twist.twist.angular.z = 0.0
        
        # Publish message
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        odometry = TractorOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass