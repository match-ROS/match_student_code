#!/usr/bin/env python3

class mecanum_kinematic:
    def __init__(self, wheel_radius, wheel_base, track_width):
        self.wr = wheel_radius
        self.wb = wheel_base
        self.tw = track_width
        
    def calc_odometrie(self, dtheta_fl, dtheta_fr, dtheta_rl, dtheta_rr, dt_encoder):
        omega_fl = dtheta_fl/dt_encoder
        omega_fr = dtheta_fr/dt_encoder
        omega_rl = dtheta_rl/dt_encoder
        omega_rr = dtheta_rr/dt_encoder
        
        L = self.wb/2
        W = self.tw/2
        
        x_dot   = self.wr / 4 * (omega_fl + omega_fr + omega_rl + omega_rr)
        y_dot   = self.wr / 4 * (-omega_fl + omega_fr + omega_rl + -omega_rr)
        psi_dot = self.wr / 4 * (-omega_fl/(L+W) + omega_fr/(L+W) + -omega_rl/(L+W) + omega_rr)
        
        return x_dot, y_dot, psi_dot