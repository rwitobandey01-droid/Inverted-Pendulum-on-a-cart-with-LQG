import numpy as np
from scipy.linalg import solve_continuous_are #solves the ARE equation

def compute_kalman_gain(A, C, Qk, Rk):
    """
    Compute the Kalman gain matrix K.

    Parameters:
    A (ndarray): State transition matrix.
    C (ndarray): Measurement matrix.
    Qk (ndarray): Process noise covariance.
    Rk (ndarray): Measurement noise covariance.

    Returns:
    L (ndarray): Kalman gain matrix.
    """
    # Solve the continuous-time algebraic Riccati equation for estimation error covariance
    P = solve_continuous_are(A.T, C.T, Qk, Rk)

    # Compute the Kalman gain
    L = P @ C.T @ np.linalg.inv(Rk)

    return L