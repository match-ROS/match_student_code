#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

class ZKalmanFilter:
    """
    A 1D Kalman Filter estimating:
        x = [ z, v_z, b_z ]^T
    where
        z    = vertical position
        v_z  = vertical velocity
        b_z  = accelerometer bias in z
    We use discrete-time propagation:
        z_{k+1}   = z_k + dt * v_z
        v_z{k+1}  = v_z{k} + dt * (a_z - b_z)
        b_z{k+1}  = b_z{k}         (bias modeled as constant or slow random walk)

    We handle a "zero-velocity" measurement, i.e. v_z = 0, which helps anchor the filter.
    """
    def __init__(self, 
                dt=0.01, 
                process_noise_acc=0.1, 
                process_noise_bias=1e-4, 
                meas_noise_vel=0.01):
        """
        Args:
            dt: Sample time (seconds)
            process_noise_acc: Std dev for unmodeled acceleration noise
            process_noise_bias: Std dev for random walk of bias
            meas_noise_vel: Std dev for velocity measurement (zero-velocity update)
        """

        self.dt = dt
        
        # State: [z, v_z, b_z]
        self.x = np.zeros((3, 1))  # Start at z=0, v=0, b=0 (can be changed)

        # State covariance
        self.P = np.eye(3) * 1.0   # Large initial uncertainty if you prefer

        # Process noise covariance Q
        # We treat unmodeled accel noise and bias random-walk as main sources.
        self.Q = np.eye(3)
        # z -> small or zero, velocity -> depends on acceleration noise, bias -> small random walk
        self.Q[0, 0] = 0.0  # Usually, direct position noise is small
        self.Q[1, 1] = (process_noise_acc * self.dt)**2
        self.Q[2, 2] = (process_noise_bias * self.dt)**2

        # Measurement noise (for v_z = 0 updates)
        self.R_vel = (meas_noise_vel)**2

        # State transition (F) and input (G) matrices in discrete time
        # x_{k+1} = F * x_k + G * a_z
        self.F = np.array([
            [1.0, self.dt, 0.0],
            [0.0, 1.0,    -self.dt],
            [0.0, 0.0,     1.0   ]
        ])
        self.G = np.array([
            [0.0],
            [self.dt],
            [0.0  ]
        ])

        # For measurement update of v_z, H = [0 1 0]
        self.H_vel = np.array([[0.0, 1.0, 0.0]])
        
    def predict(self, a_z):
        """
        Prediction step given the current accelerometer reading a_z (already minus gravity).
        """
        # x_{k+1}^- = F * x_k + G * a_z
        self.x = self.F @ self.x + self.G * a_z

        # Covariance predict:
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_zero_velocity(self):
        """
        Update step if we have a measurement that v_z = 0.
        """
        # Measurement matrix
        H = self.H_vel
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_vel
        # Kalman gain
        K = (self.P @ H.T) / S
        # Innovation = z_meas - H*x  = 0 - v_z
        y = 0.0 - (H @ self.x)[0]

        # Update state
        self.x = self.x + K * y
        # Update covariance with Joseph form or standard form
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

    def get_state(self):
        """
        Returns the current state [z, v_z, b_z].
        """
        return self.x.flatten()
