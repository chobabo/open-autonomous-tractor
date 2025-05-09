#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys
import tty
import termios
import select
import threading

class TractorTeleop:
    def __init__(self):
        rospy.init_node('tractor_teleop')
        
        # Get tractor model parameters
        tractor_model = rospy.get_param('~tractor_model', 'tractor_model')
        self.max_steering = rospy.get_param(f'/{tractor_model}/steering/max_angle', 0.785)
        
        # Parameters
        self.max_speed = rospy.get_param('~max_speed', 2.0)  # Maximum speed (m/s)
        self.speed_step = rospy.get_param('~speed_step', 0.1)  # Speed change increment
        self.steering_step = rospy.get_param('~steering_step', 0.05)  # Steering angle change increment
        
        # Current speed and steering angle
        self.current_speed = 0.0
        self.current_steering = 0.0
        
        # Publishers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.steering_pub = rospy.Publisher('/steering_angle', Float64, queue_size=10)
        
        # Exit flag
        self.done = False
        
        # Print instructions
        self.print_instructions()
    
    def print_instructions(self):
        """Print usage instructions"""
        print("\nTractor teleoperation node started. Use the following keys:")
        print("---------------------------------------------")
        print("w/s: Increase/decrease speed")
        print("a/d: Steer left/right")
        print("Space bar: Stop")
        print("q: Exit program")
        print("---------------------------------------------")
        self.print_status()
    
    def print_status(self):
        """Print current status"""
        print(f"\rSpeed: {self.current_speed:.2f} m/s | Steering angle: {self.current_steering*57.3:.1f} degrees        ", end='')
        sys.stdout.flush()
    
    def get_key(self):
        """Get keyboard input"""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Change terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            # Wait for input (timeout 0.1 seconds)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key
    
    def key_loop(self):
        """Key input processing loop"""
        while not rospy.is_shutdown() and not self.done:
            key = self.get_key()
            
            if key == 'w':  # Increase speed
                self.current_speed += self.speed_step
            elif key == 's':  # Decrease speed
                self.current_speed -= self.speed_step
            elif key == 'a':  # Steer left
                self.current_steering += self.steering_step
            elif key == 'd':  # Steer right
                self.current_steering -= self.steering_step
            elif key == ' ':  # Stop
                self.current_speed = 0.0
            elif key == 'q':  # Exit
                self.done = True
                rospy.signal_shutdown("User exit request")
                break
            
            # Limit values
            self.current_speed = max(-self.max_speed, min(self.current_speed, self.max_speed))
            self.current_steering = max(-self.max_steering, min(self.current_steering, self.max_steering))
            
            if key:  # Only print status when there's key input
                self.print_status()
    
    def publish_command(self):
        """Command message publishing loop"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown() and not self.done:
            # Twist message
            vel_msg = Twist()
            vel_msg.linear.x = self.current_speed
            self.vel_pub.publish(vel_msg)
            
            # Steering angle message
            steering_msg = Float64()
            steering_msg.data = self.current_steering
            self.steering_pub.publish(steering_msg)
            
            rate.sleep()
    
    def run(self):
        """Main execution function"""
        # Process key input in a separate thread
        key_thread = threading.Thread(target=self.key_loop)
        key_thread.daemon = True
        key_thread.start()
        
        # Publish commands in the main thread
        self.publish_command()

if __name__ == '__main__':
    try:
        teleop = TractorTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nProgram terminated.")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))