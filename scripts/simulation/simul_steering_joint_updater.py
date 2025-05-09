#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class SteeringJointUpdater:
    def __init__(self):
        rospy.init_node('steering_joint_updater')
        
        # Tractor model parameters
        self.tractor_model = rospy.get_param('~tractor_model', 'tractor_model')
        
        # Parameters (with precise namespace specification)
        self.wheelbase = rospy.get_param(f'/{self.tractor_model}/wheelbase', 2.4)  # Wheelbase
        self.front_tread_width = rospy.get_param(f'/{self.tractor_model}/front_tread_width', 1.44)  # Front tread width
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Publication rate (Hz)
        
        rospy.loginfo(f"Loaded parameters for tractor model '{self.tractor_model}':")
        rospy.loginfo(f"  wheelbase: {self.wheelbase}m")
        rospy.loginfo(f"  front_tread_width: {self.front_tread_width}m")
        rospy.loginfo(f"  publish_rate: {self.publish_rate}Hz")
        
        # Current steering angle
        self.steering_angle = 0.0
        
        # Subscriber
        rospy.Subscriber('/steering_angle', Float64, self.steering_callback)
        
        # Publisher - using separate topic (to avoid conflicts)
        self.joint_pub = rospy.Publisher('/steering_js', JointState, queue_size=10)
        
        # Timer setup
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_joint_state)
        
        rospy.loginfo("Steering joint updater node started")
    
    def steering_callback(self, msg):
        """Steering angle callback"""
        self.steering_angle = msg.data
    
    def calculate_ackermann_steering(self, steering_angle):
        """Calculate Ackermann steering angles"""
        # If steering angle is very small, return the same angle for both wheels
        if abs(steering_angle) < 0.001:
            return steering_angle, steering_angle
        
        # Calculate turning radius
        R = self.wheelbase / math.tan(abs(steering_angle))
        
        # Calculate left and right wheel rotation angles
        if steering_angle > 0:  # Right turn
            # Left wheel (inner) rotates at a larger angle
            left_angle = math.atan(self.wheelbase / (R - self.front_tread_width/2))
            # Right wheel (outer) rotates at a smaller angle
            right_angle = math.atan(self.wheelbase / (R + self.front_tread_width/2))
        else:  # Left turn
            # Right wheel (inner) rotates at a larger angle
            right_angle = -math.atan(self.wheelbase / (R - self.front_tread_width/2))
            # Left wheel (outer) rotates at a smaller angle
            left_angle = -math.atan(self.wheelbase / (R + self.front_tread_width/2))
        
        return left_angle, right_angle
    
    def publish_joint_state(self, event):
        """Publish JointState message"""
        # Calculate Ackermann steering angles
        left_angle, right_angle = self.calculate_ackermann_steering(self.steering_angle)
        
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        # Set joint states for both wheels
        msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint']
        msg.position = [left_angle, right_angle]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        
        # Publish the message
        self.joint_pub.publish(msg)

if __name__ == '__main__':
    try:
        updater = SteeringJointUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass