#!/usr/bin/env python3
import numpy as np

class tracked_kinematic:
    def __init__(self, wheel_radius, wheel_base, track_width):
        self.wr = wheel_radius
        self.wb = wheel_base
        self.tw = track_width
        
    def calc_odometrie(self, dtheta_fl, dtheta_fr, dtheta_rl, dtheta_rr, dt_encoder, yaw):
        d_left = self.wr * (dtheta_fl + dtheta_rl) / 2.0
        d_right = self.wr * (dtheta_fr + dtheta_rr) / 2.0
        
        d = (d_left + d_right) / 2.0

        dx = d * np.cos(yaw)  # X position based on heading angle
        dy = d * np.sin(yaw)  # Y position based on heading angle
        
        return dx, dy, 0