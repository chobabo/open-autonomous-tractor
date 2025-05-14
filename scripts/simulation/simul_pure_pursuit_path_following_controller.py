#!/usr/bin/env python3

import rospy
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

# Import our custom controllers
import sys
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.realpath(__file__))

# Add the control directory to Python path
control_dir = os.path.join(os.path.dirname(script_dir), 'control')
if control_dir not in sys.path:
    sys.path.append(control_dir)

# Now import the controllers
from pure_pursuit import PurePursuit
from pid_controller import VelocityPIDController


class PurePursuitPathFollowingController:
    """
    Pure Pursuit based path following controller for tractor simulation
    
    This controller uses the Pure Pursuit algorithm for lateral control
    combined with a PID controller for longitudinal (speed) control.
    
    Updated to work with map-relative coordinate system for both 
    odometry and path data.
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('pure_pursuit_path_following_controller', anonymous=True)
        
        # Get parameters
        tractor_model = rospy.get_param('~tractor_model', 'tractor_model1')
        
        # Pure Pursuit parameters
        pp_k = rospy.get_param('~pp_k', 1.0)
        pp_ks = rospy.get_param('~pp_ks', 0.5)
        wheelbase = rospy.get_param(f'/{tractor_model}/wheelbase', 2.4)
        
        # PID parameters
        pid_kp = rospy.get_param('~pid_kp', 2.0)
        pid_kd = rospy.get_param('~pid_kd', 0.1)
        pid_ki = rospy.get_param('~pid_ki', 0.01)
        
        # Control parameters
        self.target_speed = rospy.get_param('~target_speed', 1.0)  # m/s
        self.max_speed = rospy.get_param('~max_speed', 2.0)
        self.max_acceleration = rospy.get_param('~max_acceleration', 2.0)
        self.max_steering = rospy.get_param(f'/{tractor_model}/steering/max_angle', 0.785)
        
        # Goal tolerance
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)  # meters
        
        # Debug parameters
        self.debug_mode = rospy.get_param('~debug_mode', True)
        self.enable_debug_publishers = rospy.get_param('~publish_debug_info', True)
        
        # Initialize controllers
        self.pure_pursuit = PurePursuit(k=pp_k, ks=pp_ks, L=wheelbase)
        self.velocity_controller = VelocityPIDController(
            Kp=pid_kp, 
            Ki=pid_ki, 
            Kd=pid_kd,
            max_acceleration=self.max_acceleration,
            max_velocity=self.max_speed
        )
        
        # State variables
        self.current_pose = None
        self.current_velocity = 0.0
        self.path = None
        self.path_x = []
        self.path_y = []
        self.last_time = None
        self.goal_reached = False
        self.controller_active = False
        
        # Control loop timing
        self.control_frequency = rospy.get_param('~control_frequency', 10.0)  # Hz
        
        # Publishers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.steering_pub = rospy.Publisher('/steering_angle', Float64, queue_size=10)
        
        # Debug publishers
        if self.enable_debug_publishers:
            self.target_pub = rospy.Publisher('/pure_pursuit/target_point', PointStamped, queue_size=10)
            self.debug_marker_pub = rospy.Publisher('/pure_pursuit/debug_marker', Marker, queue_size=10)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/global_path', Path, self.path_callback)
        
        # Wait a bit for initial connections
        rospy.sleep(1.0)
        
        # Check if essential topics are available
        self.check_topics()
        
        # Control timer
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_frequency), 
            self.control_callback
        )
        
        rospy.loginfo("Pure Pursuit Path Following Controller initialized")
        rospy.loginfo(f"Pure Pursuit: k={pp_k}, ks={pp_ks}, L={wheelbase}")
        rospy.loginfo(f"Velocity PID: Kp={pid_kp}, Kd={pid_kd}, Ki={pid_ki}")
        rospy.loginfo(f"Target speed: {self.target_speed} m/s, Max speed: {self.max_speed} m/s")
        rospy.loginfo(f"Control frequency: {self.control_frequency} Hz")
        rospy.loginfo(f"Debug mode: {self.debug_mode}")
        rospy.loginfo("Using map-relative coordinate system for both odometry and path")
    
    def check_topics(self):
        """Check if required topics are available"""
        topics = rospy.get_published_topics()
        odom_available = any('/odom' in topic[0] for topic in topics)
        path_available = any('/global_path' in topic[0] for topic in topics)
        
        if not odom_available:
            rospy.logwarn("No /odom topic found.")
        
        if not path_available:
            rospy.logwarn("No /global_path topic found.")
    
    def odom_callback(self, msg):
        """Process odometry message (in map-relative coordinates)"""
        self.current_pose = msg.pose.pose
        
        # Extract linear velocity
        self.current_velocity = msg.twist.twist.linear.x
        
        if self.debug_mode and self.current_pose:
            x, y, yaw = self.get_current_position_orientation()
            if x is not None:
                rospy.logdebug_throttle(2.0, f"Current position (map-rel): ({x:.2f}, {y:.2f}), yaw: {yaw:.2f}, v: {self.current_velocity:.2f}")
        
    def path_callback(self, msg):
        """Process global path message (in map-relative coordinates)"""
        self.path = msg
        self.path_x = []
        self.path_y = []
        
        # Extract path points (no coordinate transformation needed)
        for pose in msg.poses:
            self.path_x.append(pose.pose.position.x)
            self.path_y.append(pose.pose.position.y)
            
        # Only show this message once to avoid spam
        if not self.controller_active:
            rospy.loginfo(f"Received path with {len(self.path_x)} points (map-relative)")
            if self.debug_mode and len(self.path_x) > 0:
                rospy.loginfo(f"Path start: ({self.path_x[0]:.2f}, {self.path_y[0]:.2f})")
                rospy.loginfo(f"Path end: ({self.path_x[-1]:.2f}, {self.path_y[-1]:.2f})")
                # Log current position for comparison
                if self.current_pose:
                    x, y, _ = self.get_current_position_orientation()
                    rospy.loginfo(f"Current position: ({x:.2f}, {y:.2f})")
                    start_dist = np.sqrt((x - self.path_x[0])**2 + (y - self.path_y[0])**2)
                    rospy.loginfo(f"Distance to path start: {start_dist:.2f}m")
        
        self.goal_reached = False
        self.controller_active = True
        self.velocity_controller.reset()
    
    def get_current_position_orientation(self):
        """Extract current position and orientation from pose"""
        if self.current_pose is None:
            return None, None, None
            
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        # Convert quaternion to euler angles
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        return x, y, yaw
    
    def check_goal_reached(self, x, y):
        """Check if goal is reached"""
        if not self.path_x or not self.path_y:
            return False
            
        # Check distance to final point
        goal_x = self.path_x[-1]
        goal_y = self.path_y[-1]
        distance_to_goal = np.sqrt((x - goal_x)**2 + (y - goal_y)**2)
        
        if self.debug_mode:
            rospy.logdebug_throttle(2.0, f"Distance to goal: {distance_to_goal:.2f}m")
        
        return distance_to_goal < self.goal_tolerance
    
    def find_closest_point(self, x, y):
        """Find closest point on path for better path following"""
        if not self.path_x or not self.path_y:
            return 0
            
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(zip(self.path_x, self.path_y)):
            dist = np.sqrt((x - px)**2 + (y - py)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Avoid looking backwards - add a small forward offset
        # This prevents the controller from getting stuck at already passed points
        forward_offset = min(10, len(self.path_x) // 10)  # Look forward by 10 points or 10% of path
        forward_idx = min(closest_idx + forward_offset, len(self.path_x) - 1)
        
        return forward_idx
    
    def publish_debug_markers(self, x, y, target_x, target_y, steering_angle):
        """Publish debug information for visualization"""
        if not self.enable_debug_publishers:
            return
            
        # Use 'map' frame for visualization since RViz typically uses map frame
        # Even though we're working with map-relative coordinates, the frame_id should be 'map'
        frame_id = "map"
        
        # Publish target point
        target_point = PointStamped()
        target_point.header.stamp = rospy.Time.now()
        target_point.header.frame_id = frame_id
        target_point.point.x = target_x
        target_point.point.y = target_y
        target_point.point.z = 0.0
        self.target_pub.publish(target_point)
        
        # Publish debug marker
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit_debug"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        
        # Direction from current position to target
        dx = target_x - x
        dy = target_y - y
        yaw = math.atan2(dy, dx)
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.debug_marker_pub.publish(marker)
    
    def control_callback(self, event):
        """Main control loop using Pure Pursuit algorithm"""
        if not self.controller_active:
            return
            
        if self.current_pose is None or not self.path_x or not self.path_y:
            if self.debug_mode:
                rospy.logwarn_throttle(5.0, "Waiting for pose or path...")
                if self.current_pose is None:
                    rospy.logwarn_throttle(10.0, "No odometry data received. Check /odom topic and simulation.")
                if not self.path_x:
                    rospy.logwarn_throttle(10.0, "No path data received. Check /global_path topic.")
            return
            
        # Get current state (in map-relative coordinates)
        x, y, yaw = self.get_current_position_orientation()
        if x is None:
            return
            
        # Check if goal is reached
        if self.check_goal_reached(x, y):
            if not self.goal_reached:
                rospy.loginfo("Goal reached! Stopping Pure Pursuit controller...")
                self.goal_reached = True
                self.controller_active = False
            
            # Stop the tractor
            self.publish_zero_command()
            return
        
        # Find closest point and create remaining path
        closest_idx = self.find_closest_point(x, y)
        remaining_path_x = self.path_x[closest_idx:]
        remaining_path_y = self.path_y[closest_idx:]
        
        if self.debug_mode:
            rospy.logdebug_throttle(2.0, f"Closest index: {closest_idx}, Remaining points: {len(remaining_path_x)}")
        
        # Calculate time step
        current_time = rospy.Time.now()
        if self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
        else:
            dt = 1.0 / self.control_frequency
        self.last_time = current_time
        
        # Ensure minimum dt to avoid division by zero
        dt = max(dt, 1e-6)
        
        # Pure Pursuit for steering control (all in map-relative coordinates)
        target_x, target_y, steering_angle = self.pure_pursuit.feedback(
            x, y, yaw, self.current_velocity, 
            remaining_path_x, remaining_path_y,
            max_steering=self.max_steering
        )
        
        # Check if we got a valid target
        if target_x is None or target_y is None:
            rospy.logwarn("No valid target point found!")
            self.publish_zero_command()
            return
        
        # Velocity PID control
        velocity_command = self.velocity_controller.velocity_feedback(
            self.target_speed, self.current_velocity, dt
        )
        
        # Publish commands
        self.publish_command(velocity_command, steering_angle)
        
        # Publish debug info
        self.publish_debug_markers(x, y, target_x, target_y, steering_angle)
        
        # Debug logging
        if self.debug_mode:
            rospy.loginfo_throttle(1.0, 
                f"Current (map-rel): ({x:.2f}, {y:.2f}, {yaw*180/np.pi:.1f}°) | "
                f"Target (map-rel): ({target_x:.2f}, {target_y:.2f}) | "
                f"Speed: {self.current_velocity:.2f}->{velocity_command:.2f} | "
                f"Steering: {steering_angle*180/np.pi:.1f}°"
            )
    
    def publish_command(self, linear_velocity, steering_angle):
        """Publish velocity and steering commands"""
        # Ensure commands are finite
        if not (np.isfinite(linear_velocity) and np.isfinite(steering_angle)):
            rospy.logwarn(f"Invalid command: v={linear_velocity}, steer={steering_angle}")
            return
            
        # Publish velocity command
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        self.vel_pub.publish(vel_msg)
        
        # Publish steering command
        steering_msg = Float64()
        steering_msg.data = steering_angle
        self.steering_pub.publish(steering_msg)
    
    def publish_zero_command(self):
        """Stop the tractor"""
        self.publish_command(0.0, 0.0)
    
    def get_controller_info(self):
        """Get information about current Pure Pursuit controller states"""
        return {
            'controller_type': 'Pure Pursuit',
            'pure_pursuit': self.pure_pursuit.get_info(),
            'velocity_pid': self.velocity_controller.get_info(),
            'current_velocity': self.current_velocity,
            'target_speed': self.target_speed,
            'goal_reached': self.goal_reached,
            'controller_active': self.controller_active,
            'coordinate_system': 'map-relative'
        }


if __name__ == '__main__':
    try:
        controller = PurePursuitPathFollowingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Pure Pursuit path following controller error: {e}")
        import traceback
        traceback.print_exc()