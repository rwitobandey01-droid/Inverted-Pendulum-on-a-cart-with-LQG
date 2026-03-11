import numpy as np
from scipy.linalg import solve_continuous_are #solves the ARE ewuation


def compute_lqr(A, B, Q, R):
    """
    Compute the optimal LQR gain matrix K.

    Parameters:
    A (ndarray): State transition matrix.
    B (ndarray): Control input matrix.
    Q (ndarray): State cost matrix.
    R (ndarray): Control cost matrix.

    Returns:
    K (ndarray): Optimal gain matrix.
    """
    # Solve the continuous-time algebraic Riccati equation
    P = solve_continuous_are(A, B, Q, R) #same as >>lqr(A, B, Q, R)[0] in MATLAB

    # Compute the optimal gain matrix K
    K = np.linalg.inv(R) @ B.T @ P #check in LQR control theory, K = R^-1 * B^T * P

    return K