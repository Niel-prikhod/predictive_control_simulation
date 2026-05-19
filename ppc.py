from sylvester import place_sylvester
import numpy as np


class PolePlacementRegulator:
    """
    Discrete-time state feedback + observer regulator.
    Uses current output y and reference r to compute control u.
    """

    def __init__(self, A_d, B_d, C_d, controller_poles, observer_poles):
        """
        A_d, B_d, C_d: discrete state-space matrices
        controller_poles: list of desired controller eigenvalues
        observer_poles: list of desired observer eigenvalues
        """
        self.A = A_d
        self.B = B_d
        self.C = C_d
        n = A_d.shape[0]
        self.K = place_sylvester(A_d, B_d, controller_poles)
        self.L = place_sylvester(A_d.T, C_d.T, observer_poles).T
        M = np.linalg.inv(np.eye(n) - (A_d - B_d @ self.K)) @ B_d
        self.N = 1.0 / (C_d @ M).item()
        self.x_hat = np.zeros((n, 1))
        self.u_prev = 0.0

    def regulate(self, output, reference):
        """
        output: measured y_k (scalar)
        reference: desired r_k (scalar)
        Returns: control signal u_k (scalar)
        """
        y = np.array([[output]])
        r = reference
        x_hat_corrected = self.x_hat + self.L @ (y - self.C @ self.x_hat)
        u = -self.K @ x_hat_corrected + self.N * r
        u = u.item()
        self.x_hat = self.A @ x_hat_corrected + self.B * u
        self.u_prev = u

        return u
