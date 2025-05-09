#!/usr/bin/env python3

import rospy
import math
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class TractorOdometry:
    def __init__(self):
        rospy.init_node('tractor_odometry_publisher')
        
        # Check tractor model type (default: tractor_model)
        tractor_model = rospy.get_param('~tractor_model', 'tractor_model')
        
        # Get tractor parameters
        self.wheelbase = rospy.get_param(f'/{tractor_model}/wheelbase', 2.4)  # Wheelbase(m)
        self.max_steering_angle = rospy.get_param(f'/{tractor_model}/steering/max_angle', 0.785)  # Max steering angle(rad)
        
        rospy.loginfo(f"Loaded tractor model parameters: wheelbase={self.wheelbase}m, max_steering={self.max_steering_angle}rad")
        
        # Other parameters
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Publication rate(Hz)
        
        # Current position and orientation
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Current velocity and steering angle
        self.velocity = 0.0  # Forward velocity(m/s)
        self.steering_angle = 0.0  # Steering angle(rad)
        
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
    
    def cmd_vel_callback(self, msg):
        """Velocity command callback"""
        # Extract forward velocity from Twist message
        self.velocity = msg.linear.x
        if self.debug:
            rospy.loginfo(f"Received velocity command: {self.velocity} m/s")
    
    def steering_callback(self, msg):
        """Steering angle callback"""
        self.steering_angle = msg.data
        if self.debug:
            rospy.loginfo(f"Received steering angle command: {self.steering_angle} rad ({math.degrees(self.steering_angle):.1f}°)")
    
    def update_odometry(self, event):
        """Periodically update odometry"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        if dt > 0 and abs(self.velocity) > 0.001:  # Only update if velocity is significant
            # Apply kinematic equations for car-like robot (Ackermann model)
            if abs(self.steering_angle) < 0.001:  # Almost straight steering
                # Straight motion
                delta_x = self.velocity * dt * math.cos(self.yaw)
                delta_y = self.velocity * dt * math.sin(self.yaw)
                delta_yaw = 0.0
                
                if self.debug:
                    rospy.loginfo(f"Straight movement: dx={delta_x:.3f}, dy={delta_y:.3f}, dyaw={delta_yaw:.3f}")
            else:
                # Turning motion
                # Calculate turning radius
                turning_radius = self.wheelbase / math.tan(abs(self.steering_angle))
                
                # Angular velocity around turning center
                angular_velocity = self.velocity / turning_radius
                if self.steering_angle < 0:
                    angular_velocity = -angular_velocity
                
                # Calculate position and orientation changes
                delta_yaw = angular_velocity * dt
                
                # Calculate position change according to Ackermann model
                # Arc movement based on turning center
                if abs(delta_yaw) > 0.001:  # If orientation change is significant
                    # Use arc movement formula
                    delta_x = (turning_radius * math.sin(self.yaw + delta_yaw) - 
                               turning_radius * math.sin(self.yaw))
                    delta_y = (turning_radius * math.cos(self.yaw) - 
                               turning_radius * math.cos(self.yaw + delta_yaw))
                    
                    # Adjust sign (based on steering direction)
                    if self.steering_angle < 0:
                        delta_x = -delta_x
                        delta_y = -delta_y
                else:
                    # Approximate small rotations as straight lines
                    delta_x = self.velocity * dt * math.cos(self.yaw)
                    delta_y = self.velocity * dt * math.sin(self.yaw)
                
                if self.debug:
                    rospy.loginfo(f"Turning movement: r={turning_radius:.3f}, ω={angular_velocity:.3f}, " +
                                 f"dx={delta_x:.3f}, dy={delta_y:.3f}, dyaw={delta_yaw:.3f}")
            
            # Update current position and orientation
            self.x += delta_x
            self.y += delta_y
            self.yaw += delta_yaw
            
            # Normalize yaw angle (-π to π)
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            
            if self.debug:
                rospy.loginfo(f"Updated position: x={self.x:.3f}, y={self.y:.3f}, " +
                             f"yaw={self.yaw:.3f} rad ({math.degrees(self.yaw):.1f}°)")
            
            # Publish odometry message
            self.publish_odometry(current_time)
        elif dt > 0:
            # Publish odometry message with latest timestamp even if velocity is 0
            self.publish_odometry(current_time)
    
    def publish_odometry(self, timestamp):
        """Publish Odometry message and TF"""
        # 1. Publish TF (odom → base_footprint)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        
        # Set position
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
        
        # 2. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Set position
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