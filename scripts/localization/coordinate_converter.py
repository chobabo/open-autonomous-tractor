#!/usr/bin/env python3

import numpy as np
import math
import rospy

class CoordinateConverter:
    """
    Class that handles conversion between plane rectangular coordinate system and latitude/longitude
    Implements the Japanese plane rectangular coordinate system conversion
    References: 
    - https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
    - https://www.gsi.go.jp/sokuchikijun/jpc.html
    """
    
    def __init__(self, config=None):
        """
        Initialize coordinate converter
        
        Args:
            config: Coordinate system conversion related settings
        """
        # Load parameters from configuration file or use default values
        if config is None:
            self.m0 = 0.9999
            self.a = 6378137.0
            self.F = 298.257222101
        else:
            self.m0 = config.get('m0', 0.9999)
            self.a = config.get('a', 6378137.0)
            self.F = config.get('F', 298.257222101)
        
        # Cache frequently used calculation results
        self.n = 1.0 / (2 * self.F - 1)
        self._init_arrays()
    
    def _init_arrays(self):
        """Initialize frequently used arrays"""
        self.A_arr = self._A_array(self.n)
        self.alpha_arr = self._alpha_array(self.n)
        self.beta_arr = self._beta_array(self.n)
        self.delta_arr = self._delta_array(self.n)
    
    def _A_array(self, n):
        """Calculate A array"""
        A0 = 1 + (n**2)/4. + (n**4)/64.
        A1 = - (3./2)*( n - (n**3)/8. - (n**5)/64. ) 
        A2 = (15./16)*( n**2 - (n**4)/4. )
        A3 = - (35./48)*( n**3 - (5./16)*(n**5) )
        A4 = (315./512)*( n**4 )
        A5 = -(693./1280)*( n**5 )
        return np.array([A0, A1, A2, A3, A4, A5])

    def _alpha_array(self, n):
        """Calculate alpha array"""
        a0 = np.nan
        a1 = (1./2)*n - (2./3)*(n**2) + (5./16)*(n**3) + (41./180)*(n**4) - (127./288)*(n**5)
        a2 = (13./48)*(n**2) - (3./5)*(n**3) + (557./1440)*(n**4) + (281./630)*(n**5)
        a3 = (61./240)*(n**3) - (103./140)*(n**4) + (15061./26880)*(n**5)
        a4 = (49561./161280)*(n**4) - (179./168)*(n**5)
        a5 = (34729./80640)*(n**5)
        return np.array([a0, a1, a2, a3, a4, a5])

    def _beta_array(self, n):
        """Calculate beta array"""
        b0 = np.nan
        b1 = (1./2)*n - (2./3)*(n**2) + (37./96)*(n**3) - (1./360)*(n**4) - (81./512)*(n**5)
        b2 = (1./48)*(n**2) + (1./15)*(n**3) - (437./1440)*(n**4) + (46./105)*(n**5)
        b3 = (17./480)*(n**3) - (37./840)*(n**4) - (209./4480)*(n**5)
        b4 = (4397./161280)*(n**4) - (11./504)*(n**5)
        b5 = (4583./161280)*(n**5)
        return np.array([b0, b1, b2, b3, b4, b5])

    def _delta_array(self, n):
        """Calculate delta array"""
        d0 = np.nan
        d1 = 2.*n - (2./3)*(n**2) - 2.*(n**3) + (116./45)*(n**4) + (26./45)*(n**5) - (2854./675)*(n**6)
        d2 = (7./3)*(n**2) - (8./5)*(n**3) - (227./45)*(n**4) + (2704./315)*(n**5) + (2323./945)*(n**6)
        d3 = (56./15)*(n**3) - (136./35)*(n**4) - (1262./105)*(n**5) + (73814./2835)*(n**6)
        d4 = (4279./630)*(n**4) - (332./35)*(n**5) - (399572./14175)*(n**6)
        d5 = (4174./315)*(n**5) - (144838./6237)*(n**6)
        d6 = (601676./22275)*(n**6)
        return np.array([d0, d1, d2, d3, d4, d5, d6])
    
    def calc_xy(self, phi_deg, lambda_deg, phi0_deg, lambda0_deg):
        """
        Convert latitude/longitude to plane rectangular coordinate system X, Y coordinates
        
        Args:
            phi_deg: Latitude to convert (degrees)
            lambda_deg: Longitude to convert (degrees)
            phi0_deg: Latitude of plane rectangular coordinate system origin (degrees)
            lambda0_deg: Longitude of plane rectangular coordinate system origin (degrees)
            
        Returns:
            tuple: (X, Y) coordinates (meters)
        """
        try:
            phi_rad = np.deg2rad(phi_deg)
            lambda_rad = np.deg2rad(lambda_deg)
            phi0_rad = np.deg2rad(phi0_deg)
            lambda0_rad = np.deg2rad(lambda0_deg)

            A_ = ((self.m0 * self.a) / (1. + self.n)) * self.A_arr[0]
            S_ = ((self.m0 * self.a) / (1. + self.n)) * (self.A_arr[0] * phi0_rad + 
                                                        np.dot(self.A_arr[1:], np.sin(2 * phi0_rad * np.arange(1, 6))))

            lambda_c = np.cos(lambda_rad - lambda0_rad)
            lambda_s = np.sin(lambda_rad - lambda0_rad)

            t = np.sinh(np.arctanh(np.sin(phi_rad)) - 
                      ((2 * np.sqrt(self.n)) / (1 + self.n)) * 
                      np.arctanh(((2 * np.sqrt(self.n)) / (1 + self.n)) * np.sin(phi_rad)))
            t_ = np.sqrt(1 + t * t)

            xi2 = np.arctan(t / lambda_c)
            eta2 = np.arctanh(lambda_s / t_)

            x = A_ * (xi2 + np.sum(np.multiply(self.alpha_arr[1:],
                                           np.multiply(np.sin(2 * xi2 * np.arange(1, 6)),
                                                      np.cosh(2 * eta2 * np.arange(1, 6)))))) - S_
            y = A_ * (eta2 + np.sum(np.multiply(self.alpha_arr[1:],
                                            np.multiply(np.cos(2 * xi2 * np.arange(1, 6)),
                                                        np.sinh(2 * eta2 * np.arange(1, 6))))))
            return x, y
        except Exception as e:
            rospy.logerr(f"Coordinate conversion error (lat/lon → X,Y): {e}")
            return 0.0, 0.0

    def calc_lat_lon(self, x, y, phi0_deg, lambda0_deg):
        """
        Convert plane rectangular coordinate system X, Y coordinates to latitude/longitude
        
        Args:
            x: X coordinate to convert (meters)
            y: Y coordinate to convert (meters)
            phi0_deg: Latitude of plane rectangular coordinate system origin (degrees)
            lambda0_deg: Longitude of plane rectangular coordinate system origin (degrees)
            
        Returns:
            tuple: (latitude, longitude) (degrees)
        """
        try:
            phi0_rad = np.deg2rad(phi0_deg)
            lambda0_rad = np.deg2rad(lambda0_deg)
            
            A_ = ((self.m0 * self.a) / (1. + self.n)) * self.A_arr[0]
            S_ = ((self.m0 * self.a) / (1. + self.n)) * (self.A_arr[0] * phi0_rad + 
                                                        np.dot(self.A_arr[1:], np.sin(2 * phi0_rad * np.arange(1, 6))))

            xi = (x + S_) / A_
            eta = y / A_

            xi2 = xi - np.sum(np.multiply(self.beta_arr[1:], 
                                        np.multiply(np.sin(2 * xi * np.arange(1, 6)),
                                                   np.cosh(2 * eta * np.arange(1, 6)))))
            eta2 = eta - np.sum(np.multiply(self.beta_arr[1:],
                                         np.multiply(np.cos(2 * xi * np.arange(1, 6)),
                                                    np.sinh(2 * eta * np.arange(1, 6)))))

            chi = np.arcsin(np.sin(xi2) / np.cosh(eta2))
            latitude = chi + np.dot(self.delta_arr[1:], np.sin(2 * chi * np.arange(1, 7)))

            longitude = lambda0_rad + np.arctan(np.sinh(eta2) / np.cos(xi2))

            return np.rad2deg(latitude), np.rad2deg(longitude)
        except Exception as e:
            rospy.logerr(f"Coordinate conversion error (X,Y → lat/lon): {e}")
            return phi0_deg, lambda0_deg

    def transform_coordinates(self, offset, prc_x, prc_y, yaw):
        """
        Transform coordinates from one reference point (e.g., antenna) to another (e.g., rear axle)
        
        Args:
            offset: [x, y, z] offset (meters)
            prc_x: Original X coordinate (meters)
            prc_y: Original Y coordinate (meters)
            yaw: Rotation angle (radians)
            
        Returns:
            tuple: Transformed (X, Y) coordinates
        """
        try:
            trans_x = (offset[0] * math.cos(yaw)) - (offset[1] * math.sin(yaw)) + prc_x
            trans_y = (offset[0] * math.sin(yaw)) + (offset[1] * math.cos(yaw)) + prc_y
            return trans_x, trans_y
        except Exception as e:
            rospy.logerr(f"Coordinate transformation error (applying offset): {e}")
            return prc_x, prc_y

    def get_origin_points(self):
        """
        Return all origin coordinates of the Japanese plane rectangular coordinate system
        
        Returns:
            list: List of [(latitude, longitude)] origins
        """
        return [
            [0.0, 0.0],                 # 0: Not used
            [33.0, 129.5],              # 1
            [33.0, 131.0],              # 2
            [36.0, 132.16666667],       # 3
            [33.0, 133.5],              # 4
            [36.0, 134.33333333],       # 5
            [36.0, 136.0],              # 6
            [36.0, 137.16666667],       # 7
            [36.0, 138.5],              # 8
            [36.0, 139.83333333],       # 9: Ibaraki (default)
            [40.0, 140.83333333],       # 10
            [44.0, 140.25],             # 11
            [44.0, 142.25],             # 12
            [44.0, 144.25],             # 13
            [26.0, 142.0],              # 14
            [26.0, 127.5],              # 15
            [26.0, 124.0],              # 16
            [26.0, 131.0],              # 17
            [20.0, 136.0],              # 18
            [26.0, 154.0]               # 19
        ]

    def get_origin_by_zone(self, zone):
        """
        Return the origin coordinates for the specified plane rectangular coordinate system zone
        
        Args:
            zone: Plane rectangular coordinate system zone number (1-19)
            
        Returns:
            tuple: (latitude, longitude) or None if invalid zone number
        """
        if 1 <= zone <= 19:
            origin_points = self.get_origin_points()
            return origin_points[zone]
        else:
            rospy.logerr(f"Invalid plane rectangular coordinate system zone number: {zone}")
            return None