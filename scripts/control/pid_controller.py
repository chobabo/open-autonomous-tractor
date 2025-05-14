#!/usr/bin/env python3

import numpy as np

class PIDController(object):
    """
    PID Controller for longitudinal control
    
    This class implements a PID (Proportional-Integral-Derivative) controller
    for controlling vehicle speed or other continuous variables.
    """
    
    def __init__(self, Kp=1.0, Ki=0.01, Kd=0.1, max_output=None, max_integral=10.0):
        """
        Initialize PID controller
        
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain  
            Kd (float): Derivative gain
            max_output (float, optional): Maximum absolute output value
            max_integral (float): Maximum absolute integral term for anti-windup
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.max_integral = max_integral
        
        # Internal state
        self.prev_error = None
        self.integral_error = 0.0
        self.last_time = None
        
    def set_parameters(self, Kp=None, Ki=None, Kd=None, max_output=None, max_integral=None):
        """
        Update PID parameters
        
        Args:
            Kp (float, optional): Proportional gain
            Ki (float, optional): Integral gain
            Kd (float, optional): Derivative gain
            max_output (float, optional): Maximum output constraint
            max_integral (float, optional): Maximum integral term
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        if max_output is not None:
            self.max_output = max_output
        if max_integral is not None:
            self.max_integral = max_integral
    
    def reset(self):
        """
        Reset PID controller internal state
        """
        self.prev_error = None
        self.integral_error = 0.0
        self.last_time = None
    
    def feedback(self, error, dt=None, current_time=None):
        """
        Calculate PID control output
        
        Args:
            error (float): Current error (target - current)
            dt (float, optional): Time step (if not provided, calculated from current_time)
            current_time (float, optional): Current timestamp
            
        Returns:
            float: PID control output
        """
        # Handle time step calculation
        if dt is None and current_time is not None:
            if self.last_time is not None:
                dt = current_time - self.last_time
            else:
                dt = 0.0
            self.last_time = current_time
        elif dt is None:
            dt = 0.0
        
        # Initialize previous error if first run
        if self.prev_error is None:
            self.prev_error = error
            
        # Proportional term
        proportional = self.Kp * error
        
        # Integral term with anti-windup
        if dt > 0:
            self.integral_error += error * dt
            # Apply integral windup protection
            self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        integral = self.Ki * self.integral_error
        
        # Derivative term
        if dt > 0:
            derivative = self.Kd * (error - self.prev_error) / dt
        else:
            derivative = 0.0
        self.prev_error = error
        
        # Calculate total output
        output = proportional + integral + derivative
        
        # Apply output constraints if specified
        if self.max_output is not None:
            output = np.clip(output, -self.max_output, self.max_output)
        
        return output
    
    def get_components(self, error, dt=None, current_time=None):
        """
        Get individual PID components for debugging
        
        Args:
            error (float): Current error
            dt (float, optional): Time step
            current_time (float, optional): Current timestamp
            
        Returns:
            dict: Dictionary containing P, I, D components and total output
        """
        # Store original state to restore after calculation
        original_prev_error = self.prev_error
        original_integral = self.integral_error
        original_last_time = self.last_time
        
        # Calculate components
        output = self.feedback(error, dt, current_time)
        
        # Calculate individual components
        proportional = self.Kp * error
        integral = self.Ki * self.integral_error  
        derivative = self.Kd * (error - original_prev_error) / dt if dt > 0 and original_prev_error is not None else 0.0
        
        return {
            'proportional': proportional,
            'integral': integral,
            'derivative': derivative,
            'total': output,
            'error': error
        }
    
    def get_info(self):
        """
        Get current controller parameters and state
        
        Returns:
            dict: Dictionary containing current parameters and state
        """
        return {
            'Kp': self.Kp,
            'Ki': self.Ki,
            'Kd': self.Kd,
            'max_output': self.max_output,
            'max_integral': self.max_integral,
            'integral_error': self.integral_error,
            'prev_error': self.prev_error
        }


class VelocityPIDController(PIDController):
    """
    Specialized PID controller for velocity control
    
    This class extends the basic PID controller with features specific
    to velocity control applications.
    """
    
    def __init__(self, Kp=2.0, Ki=0.01, Kd=0.1, max_acceleration=2.0, max_velocity=None):
        """
        Initialize velocity PID controller
        
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
            max_acceleration (float): Maximum allowed acceleration (m/s²)
            max_velocity (float, optional): Maximum allowed velocity (m/s)
        """
        super().__init__(Kp, Ki, Kd, max_output=max_acceleration)
        self.max_velocity = max_velocity
        self.last_velocity_command = 0.0
        
    def velocity_feedback(self, target_velocity, current_velocity, dt):
        """
        Calculate velocity command with acceleration limiting
        
        Args:
            target_velocity (float): Desired velocity (m/s)
            current_velocity (float): Current velocity (m/s)
            dt (float): Time step (seconds)
            
        Returns:
            float: Commanded velocity (m/s)
        """
        # Calculate velocity error
        velocity_error = target_velocity - current_velocity
        
        # Get acceleration command from PID
        acceleration_command = self.feedback(velocity_error, dt)
        
        # Calculate new velocity command
        new_velocity_command = current_velocity + acceleration_command * dt
        
        # Apply velocity constraints
        if self.max_velocity is not None:
            new_velocity_command = np.clip(new_velocity_command, 0.0, self.max_velocity)
        
        self.last_velocity_command = new_velocity_command
        return new_velocity_command