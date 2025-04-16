#!/usr/bin/env python3
import numpy as np

class differential_kinematic:
    def __init__(self, wheel_radius, wheel_base, track_width):
        self.wr = wheel_radius
        self.wb = wheel_base
        self.tw = track_width
        
    def calc_odometrie(self, dtheta_fl, dtheta_fr, dtheta_rl, dtheta_rr, dt_encoder, yaw):
        d = (dtheta_fl + dtheta_fr) / 2.0

        dx = d * np.cos(yaw)  # X position based on heading angle
        dy = d * np.sin(yaw)  # Y position based on heading angle
        
        return dx, dy, 0