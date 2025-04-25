import numpy as np

class SpringDamperRLS:
    """
    Recursive Least Squares for identifying c/m and k/m from displacement data.
    
    State: theta = [c/m, k/m]
    """

    def __init__(self, mass, forgetting_factor=0.7, P0=1e3):
        """
        Args:
            mass (float): Known mass of the system.
            forgetting_factor (float): 'lambda' in RLS; close to 1 for slow forgetting.
            P0 (float): Initial scaling for the covariance matrix.
        """
        self.mass = mass
        self.lmbda = forgetting_factor
        # Parameter vector (c/m, k/m) initialized to zeros:
        self.theta = np.array([10.0, 100.0])
        # Covariance matrix:
        self.P = P0 * np.eye(2)

    def update(self, z, z_dot, z_ddot):
        """
        Perform one RLS update with a new measurement of (z, z_dot, z_ddot).

        z     : displacement at time t
        z_dot : velocity at time t
        z_ddot: acceleration at time t

        Returns:
            (c_est, k_est): The updated estimates of c and k.
        """
        # Construct regressor: phi(t) = [-z_dot(t), -z(t)]
        phi = np.array([z_dot, z])
        # Output y(t) = z_ddot(t)
        y = z_ddot

        # 1) Denominator for the gain:
        phi_P_phi = phi @ (self.P @ phi)  # scalar
        g_k = self.lmbda + phi_P_phi

        # 2) Kalman-like gain vector:
        K = (self.P @ phi) / g_k  # shape (2,)

        # 3) Parameter update:
        y_pred = phi @ self.theta + 9.81
        error = y - y_pred
        self.theta += K * error  # RLS core update

        # 4) Covariance update:
        self.P = (1.0 / self.lmbda) * (self.P - np.outer(K, phi) @ self.P)

        # Extract c, k from [c/m, k/m]:
        c_over_m, k_over_m = self.theta
        c_est = c_over_m * self.mass
        k_est = k_over_m * self.mass

        return c_est, k_est
