import numpy as np
from scipy.linalg import solve_sylvester


def place_sylvester(A, B, desired_poles):
    """
    Pole placement using Sylvester equation.

    Parameters:
    A: n x n system matrix
    B: n x m input matrix (m=1 for SISO)
    desired_poles: list/array of n desired closed-loop eigenvalues

    Returns:
    K: m x n state feedback gain matrix
    """
    n = A.shape[0]
    m = B.shape[1]
    Lambda = np.zeros((n, n), dtype=complex)
    i = 0
    while i < n:
        if i < n-1 and np.iscomplex(desired_poles[i]):
            Lambda[i, i] = desired_poles[i]
            Lambda[i+1, i+1] = desired_poles[i+1]
            i += 2
        else:
            Lambda[i, i] = desired_poles[i]
            i += 1
    G_bar = np.ones((m, n))
    X = solve_sylvester(A, -Lambda, -B @ G_bar)
    if np.linalg.cond(X) > 1e10:
        print("Warning: X is ill-conditioned, try different G_bar")
    K = G_bar @ np.linalg.inv(X)
    if np.allclose(np.imag(K), 0):
        K = np.real(K)
    return K
