#!/usr/bin/env python3

import numpy as np

class PurePursuit(object):
    """
    Pure Pursuit Algorithm for lateral control
    
    This class implements the Pure Pursuit algorithm for path following,
    which calculates steering commands based on a look-ahead point on the path.
    """
    
    def __init__(self, k=1.0, ks=0.1, L=2.8):
        """
        Initialize Pure Pursuit controller
        
        Args:
            k (float): Look-ahead gain (multiplied by velocity)
            ks (float): Minimum look-ahead distance
            L (float): Wheelbase length of the vehicle
        """
        self.k = k
        self.ks = ks
        self.L = L
        
    def set_parameters(self, k=None, ks=None, L=None):
        """
        Update Pure Pursuit parameters
        
        Args:
            k (float, optional): Look-ahead gain
            ks (float, optional): Minimum look-ahead distance  
            L (float, optional): Wheelbase length
        """
        if k is not None:
            self.k = k
        if ks is not None:
            self.ks = ks
        if L is not None:
            self.L = L
    
    def calculate_look_ahead_distance(self, velocity):
        """
        Calculate look-ahead distance based on current velocity
        
        Args:
            velocity (float): Current vehicle velocity
            
        Returns:
            float: Look-ahead distance
        """
        return self.k * abs(velocity) + self.ks
    
    def find_target_point(self, x, y, map_xs, map_ys, look_ahead_dist):
        """
        Find target point on the path based on look-ahead distance
        
        Args:
            x, y (float): Current vehicle position
            map_xs, map_ys (list): Path points (x, y coordinates)
            look_ahead_dist (float): Look-ahead distance
            
        Returns:
            tuple: (target_x, target_y, target_index) or (None, None, None) if not found
        """
        for i, (_x, _y) in enumerate(zip(map_xs, map_ys)):
            d = np.sqrt((x - _x)**2 + (y - _y)**2)
            if d >= look_ahead_dist:
                return _x, _y, i
        
        # If no point found, return last point
        if map_xs and map_ys:
            return map_xs[-1], map_ys[-1], len(map_xs) - 1
        else:
            return None, None, None
    
    def calculate_steering_angle(self, x, y, yaw, target_x, target_y, look_ahead_dist):
        """
        Calculate steering angle to reach target point
        
        Args:
            x, y (float): Current vehicle position
            yaw (float): Current vehicle orientation (radians)
            target_x, target_y (float): Target point coordinates
            look_ahead_dist (float): Look-ahead distance used
            
        Returns:
            float: Steering angle (radians)
        """
        # Transform target point to vehicle coordinate system
        dx = target_x - x
        dy = target_y - y
        
        # Rotate to vehicle frame
        target_x_veh = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        target_y_veh = dx * np.sin(-yaw) + dy * np.cos(-yaw)
        
        # Calculate curvature and steering angle
        curvature = 2 * target_y_veh / (look_ahead_dist**2)
        steering_angle = np.arctan(curvature * self.L)
        
        return steering_angle
    
    def feedback(self, x, y, yaw, v, map_xs, map_ys, max_steering=None):
        """
        Main feedback method for Pure Pursuit control
        
        Args:
            x, y (float): Current position
            yaw (float): Current orientation (radians)
            v (float): Current velocity
            map_xs, map_ys (list): Path points (x, y coordinates)
            max_steering (float, optional): Maximum steering angle constraint
            
        Returns:
            tuple: (target_x, target_y, steering_angle)
        """
        # Calculate look-ahead distance
        look_ahead_dist = self.calculate_look_ahead_distance(v)
        
        # Find target point
        target_x, target_y, _ = self.find_target_point(x, y, map_xs, map_ys, look_ahead_dist)
        
        if target_x is None:
            return x, y, 0.0
        
        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(x, y, yaw, target_x, target_y, look_ahead_dist)
        
        # Apply steering constraints if provided
        if max_steering is not None:
            steering_angle = np.clip(steering_angle, -max_steering, max_steering)
        
        return target_x, target_y, steering_angle
    
    def get_info(self):
        """
        Get current controller parameters
        
        Returns:
            dict: Dictionary containing current parameters
        """
        return {
            'k': self.k,
            'ks': self.ks,
            'L': self.L
        }