import numpy as np

#------------------------------
#Physical parameters
#------------------------------

PARAMS = {
    "M": 1, # mass of the cart
    "m": 0.1, # mass of the pendulum
    "l": 0.5, # length of the pendulum
    "g": 9.81, # acceleration due to gravity
    "I": 1/3 * 0.1 * 0.5**2, # moment of inertia of the pendulum I = 1/3 * m * l^2 for a rod pivoted at one end
    "b": 0.1 # damping coefficient (friction)
}

#------------------------------
#LQR parameters
#------------------------------

Q = np.diag([100, 1, 300 , 5]) # state cost matrix # aggrresive pos tracking
R = np.diag([0.7]) # control cost matrix
k_swing = 7 # swing-up gain
#___THRESHOLDS FOR CONTROLLER SWITCHING___
angle_threshold = 1
velocity_threshold = 8 #letting LQR try more :))
force_lim = 20
Ramp_rate = 1.5
#------------------------------
#initial conditions
#------------------------------

#INITIAL_STATE = np.array([ 1, 0, np.pi + 0.01, 0]) # [x, x_dot, theta, theta_dot]
INITIAL_STATE = np.array([ 0 , 0, 0, 0]) # [x, x_dot, theta, theta_dot]
#------------------------------
#Simulation parameters
#------------------------------

T_FINAL = 40    # total simulation time
MAX_STEP = 0.01 # maximum number of simulation steps