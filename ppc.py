import numpy as np


class PolePlacementRegulator:
    """
    Discrete-time state feedback + observer regulator.
    Uses current output y and reference r to compute control u.
    """

    def __init__(self, A_poly, B_poly, poles, dt):
        """
        A_poly: denominator coeffs; B_poly: numerator coeffs;
        poles: desired continuous closed-loop poles; dt: sample time.
        """
        self.dt = dt
        n = len(A_poly) - 1
        poles_discrete = [np.exp(s * dt) for s in poles]
        D_poly = np.poly(poles_discrete)
        if len(D_poly) < 2 * n:
            raise ValueError("poles: Invalid Dimension")
        self.D_poly = D_poly
        self.P_poly, self.Q_poly = self._solve_dioph_eq(
            A_poly, B_poly, D_poly, n)
        self.R = np.polyval(D_poly, 1) / np.polyval(B_poly, 1)
        self.u_prev = np.zeros(len(self.P_poly) - 1)
        self.y_prev = np.zeros(len(self.Q_poly) - 1)

    def _solve_dioph_eq(self, A_poly, B_poly, D_poly, n):
        """Solve A(z)P(z) + B(z)Q(z) = D(z) via least-squares."""
        A_coeff = np.array(A_poly)
        B_coeff = np.array(B_poly)
        matrix = np.zeros((2 * n, 2 * n - 1))
        rhs = np.zeros(2 * n)
        for k in range(2*n):
            for j in range(n-1):
                i = k - j
                if 0 <= i <= n:
                    a_i = A_coeff[n - i] if i <= n else 0.0
                    matrix[k, j] += a_i
            i_pn1 = k - (n-1)
            if 0 <= i_pn1 <= n:
                rhs[k] -= A_coeff[n - i_pn1]
            m = len(B_coeff) - 1
            for j in range(n):
                i = k - j
                if 0 <= i <= m:
                    b_i = B_coeff[m - i]
                    matrix[k, n-1 + j] += b_i
            if k < len(D_poly):
                d_k = D_poly[-1 - k] if k < len(D_poly) else 0.0
                rhs[k] += d_k
        x, residuals, rank, s = np.linalg.lstsq(matrix, rhs, rcond=None)
        p_poly = np.zeros(n)
        p_poly[0] = 1.0
        p_poly[1:] = x[:n - 1][:: - 1]
        q_poly = x[n - 1:][:: - 1]
        return p_poly, q_poly

    def regulate(self, output, reference):
        """
        output: measured y_k (scalar)
        reference: desired r_k (scalar)
        Returns: control signal u_k (scalar)
        """
        y_vec = np.concatenate([[output], self.y_prev])
        feedback = np.dot(self.Q_poly, y_vec)
        ff = self.R * reference
        u = ff - feedback - np.dot(self.P_poly[1:], self.u_prev)
        self.y_prev = np.roll(self.y_prev, 1)
        self.y_prev[0] = output
        self.u_prev = np.roll(self.u_prev, 1)
        if len(self.u_prev) > 0:
            self.u_prev[0] = u
        return u
