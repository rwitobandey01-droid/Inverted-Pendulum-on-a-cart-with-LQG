import numpy as np

def cartpole_nonlinear(t, state, u, params):
    x, x_dot, theta, theta_dot = state
    #theta = theta - np.pi # shift theta to be zero at the upright position

    u = float(u) # ensure u is a scalar

    M = params['M']     #cart mass
    m = params['m']     #pendulum mass
    l = params['l']     #pendulum length
    g = params['g']     #gravity
    I = params['I']     #moment of inertia of the pendulum
    b = params['b']     #damping coefficient (friction)

    s = np.sin(theta)
    c = np.cos(theta)


    # Denominator
    D = (M+m)*(I+m*l**2) - (m*l*c)**2
    
    # Accelerations
    x_ddot = (
           (I + m*l**2) * (u - b*x_dot)
             + m**2 * l**2 * theta_dot**2 * s
            - m**2 * g * l**2 * s * c
           ) / D

    theta_ddot = (
        -m*l*(u - b*x_dot)*c
        - m*l*(M+m)*g*s
        + m**2*l**2*theta_dot**2*s*c
    ) / D

    return np.array([x_dot, x_ddot, theta_dot, theta_ddot], dtype=float)
