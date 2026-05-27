import numpy as np


class GeneralPredictiveController:
    def __init__(self, num, den, horizon, penalty, constraints=None):
        """
        num, den: discrete plant;
        horizon: prediction horizon;
        penalty: control weight.
        """
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

        mat_G = np.linalg.solve(mat_A, mat_B)
        if constraints is None:
            self.type = 'a'
            id_mat = np.eye(horizon)
            gain_mat = (np.linalg.inv(
                mat_G.T @ mat_G + penalty * id_mat) @ mat_G.T)
            self.control_gain = gain_mat[0, :]
        else:
            self.type = 'n'
            self.hessian = mat_G.T @ mat_G + penalty * np.eye(horizon)
            self.control_min, self.control_max = constraints
            self.sum_mat = np.tril(np.ones((horizon, horizon)))
            self.constr_mat = np.vstack([self.sum_mat, -self.sum_mat])
            self.control_gain = mat_G
        self.in_resp = np.linalg.solve(mat_A, tilde_B)
        self.out_resp = np.linalg.solve(mat_A, tilde_A)

        self.out_hist = np.zeros(m)
        self.in_hist = np.zeros(n - 1)
        self.prev_control = 0.0

    def _hildreth_desop(self, lin_term, constraints, max_iter=100, tol=1e-6):
        hess_inv = np.linalg.inv(self.hessian)
        mat_g = 0.25 * self.sum_mat @ hess_inv @ self.sum_mat.T
        vec_h = 0.5 * self.sum_mat @ hess_inv @ lin_term + constraints
        constr_len = len(constraints)
        dual_vec = np.zeros(constr_len)
        for _ in range(max_iter):
            dual_prev = dual_vec.copy()
            for i in range(constr_len):
                sum = mat_g[i, :] @ dual_vec - mat_g[i, i] * dual_vec[i]
                next_elem = -(1.0 / mat_g[i, i]) * (sum + 0.5 * vec_h[i])
                dual_vec[i] = max(0, next_elem)
            if np.linalg.norm((dual_vec - dual_prev) < tol):
                break
        return (-0.5 * hess_inv @ (self.sum_mat.T @ dual_vec + lin_term))

    def regulate(self, out, ref):
        """Compute control u_k using receding-horizon GPC law."""
        self.out_hist = np.roll(self.out_hist, 1)
        self.out_hist[0] = out
        free_resp = self.in_resp @ self.in_hist + self.out_resp @ self.out_hist
        ref_vec = np.full(self.horizon, ref)
        if self.type == 'a':
            control_incr = self.control_gain @ (ref_vec - free_resp)
        else:
            max_incr = np.full(
                self.horizon, self.control_max - self.prev_control)
            min_incr = np.full(
                self.horizon, self.prev_control - self.control_min)
            constraints = np.hstack([min_incr, max_incr])
            lin_term = - self.control_gain.T @ (ref_vec - free_resp)
            control_incr = self._hildreth_desop(lin_term, constraints)
        self.in_hist = np.roll(self.in_hist, 1)
        self.in_hist[0] = control_incr
        self.prev_control += control_incr
        return self.prev_control
