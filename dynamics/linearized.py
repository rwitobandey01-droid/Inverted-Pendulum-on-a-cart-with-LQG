import numpy as np

def linear_matrices(params):
    # params is a dict; use bracket access instead of calling it
    M = params['M']     # cart mass
    m = params['m']     # pendulum mass
    l = params['l']     # pendulum length
    g = params['g']     # gravity
    I = params['I']     # moment of inertia of the pendulum
    b = params['b']     # damping coefficient (friction)
 
    D = I * (M + m) + M * m * l**2 # Denominator for the A and B matrices

    A = np.array([
                  [0, 1, 0, 0],
                  [0, -(I + m*l**2)*b/D, (m**2 * g * l**2)/D, 0],
                  [0, 0, 0, 1],
                  [0, -(m*l*b)/D, m*g*l*(M+m)/D, 0]
                ])
    B = np.array([
                  [0],
                  [(I + m*l**2)/D],
                  [0],
                  [m*l/D]                    
                ])
   
    return A, B