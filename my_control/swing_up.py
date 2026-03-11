import numpy as np
from utils.config import force_lim

def swing_up_controller(state, params, k_swing=8):

    x, x_dot, theta, theta_dot = state

    m = params["m"]
    l = params["l"]
    g = params["g"]
    I = params['I'] 

    # pendulum energy
    #E = 0.5 * m * (l**2) * theta_dot**2 + m * g * l * (1 - np.cos(theta))
    E = 0.5 * (I + m*l**2) * theta_dot**2 + m*g*l*(1 - np.cos(theta))
    
    # desired energy (upright)
    #E_des = 2 * m * g * l  # potential energy at upright, no KE needed
    E_des = 2 * m * g * l*0.90 #90% of upright energy 

    # small kick if system is stuck
    #if abs(theta_dot) < 1e-3:

    "BIGG Culprit not letting LQR start"
    #if abs(theta_dot) < 1e-3 and abs(theta -np.pi) > 0.3:
    #    return 0.2*np.sign(np.cos(theta)) #square wave ie impulse

    # energy shaping
    u = k_swing * (E - E_des) * theta_dot * np.cos(theta)

    #softer control near upright as its too fast 
    #if abs(theta - np.pi) < 0.3:
    #   u *= 0.3

    # second try ___ 
    #smooth takeover near upright - let LQR take over cleanly
    #proximity = np.clip(abs(theta - np.pi) / 0.3, 0, 1)
    #u *= proximity
    

    #third try__
    # ACTIVE BRAKING near upright — slow it down so observer catches up
    #angle_to_upright = abs(((theta - np.pi + np.pi) % (2*np.pi)) - np.pi)
    #if angle_to_upright < 0.5 and abs(theta_dot) > 2.0:
    #   u = -3.0 * np.sign(theta_dot)  # active brake
    #___never mind REEALLLYY BAD__


     # much stronger kick, only when very low energy
    if E < 0.1 * E_des:
        return 5.0 * np.sign(np.cos(theta))  # was 0.2, needs to be strong

    u = k_swing * (E - E_des) * np.sign(theta_dot * np.cos(theta))

    # cart centering
    u -= 0.3 * x

    u =np.clip(u, -force_lim, force_lim)  # actuator limit

    return u