import numpy as np


class GeneralPredictiveController:
    def __init__(self, num, den, horizon, penalty):
        self.horizon = horizon
        self.penalty = penalty
        aug_num = np.convolve(den, [1, -1])
        a_poly = aug_num[1:]
        mat_A = np.eye(horizon)
        for i in range(1, horizon):
            for j in range(i):
                idx = i - j - 1
                if idx < len(a_poly):
                    mat_A[i, j] = a_poly[idx]

        b_poly = np.array(num)
        mat_B = np.zeros((horizon, horizon))
        for i in range(horizon):
            for j in range(i + 1):
                idx = i - j
                if idx < len(b_poly):
                    mat_B[i, j] = b_poly[idx]

        m = len(a_poly)
        n = len(b_poly)
        tilde_A = np.zeros((horizon, m))
        tilde_B = np.zeros((horizon, n - 1))
        for i in range(horizon):
            for j in range(m):
                idx = i + j
                if idx < m:
                    tilde_A[i, j] = -a_poly[idx]
            for j in range(n - 1):
                idx = i + j + 1
                if idx < n:
                    tilde_B[i, j] = b_poly[idx]

        gain = np.linalg.solve(mat_A, mat_B)
        in_history = np.linalg.solve(mat_A, tilde_B)
        out_history = np.linalg.solve(mat_A, tilde_A)

    def regulate(self, out, ref):
