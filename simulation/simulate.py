import numpy as np
from scipy.integrate import solve_ivp

from dynamics.nonlinear import cartpole_nonlinear
from my_control.swing_up import swing_up_controller
from utils.config import k_swing, angle_threshold, velocity_threshold, Ramp_rate


def simulate(initial_state, K, A, B, C, L, params, t_final=30, x_ref=0):

    # ---- observer initial state ----
    x_hat0 = np.array([
        initial_state[0],           # x
        initial_state[1],           # x_dot
        initial_state[2] - np.pi,   # shift to linear frame
        initial_state[3]            # theta_dot
    ])

    Z0 = np.hstack((initial_state, x_hat0))

    # reference state
    x_ref_vec = np.array([x_ref, 0, 0, 0])
    # ---- ramp state ----
    # last_t tracks real elapsed time so ramp moves at m/s not per-call
    x_ref_ramp = {'current': initial_state[0], 'last_t':0.0, 'lqr_active': False}

    def closed_loop(t, Z):
      
        # ----- split system state -----
        state = Z[:4]
        x_hat = Z[4:] 

        x, x_dot, theta, theta_dot = state

        #shift state for controller (upright == 0)
        theta_lin = theta - np.pi

        # wrap theta error around upright
        theta_err = ((theta_lin + np.pi) % (2*np.pi)) - np.pi

        # for debugging
        in_lqr_zone = abs(theta_err) < angle_threshold and abs(theta_dot) < velocity_threshold
        
        if abs(t % 0.5) < 0.01:
            print(f"t={t:.1f} | theta_err={np.degrees(theta_err):.1f}° "
                f"| theta_dot={theta_dot:.2f} "
                f"| LQR={'YES' if in_lqr_zone else 'NO'} "
                f"| min_err={np.degrees(abs(theta_err)):.1f}°"
                f"| ref_ramp={x_ref_ramp['current']}°")
        
        # ----- measurements -----
        y = np.array([
            x,
            theta_lin
        ])

        y_pred = C @ x_hat

        
        #ramping to the postion inetead step

        #------ time based ramp ------
        if in_lqr_zone:
            dt = t - x_ref_ramp['last_t'] #time elapsed
            x_ref_ramp['last_t'] = t      #update for next call

            if x_ref_ramp['current'] < x_ref:
                x_ref_ramp['current'] = min(x_ref_ramp['current']+ Ramp_rate*dt , x_ref)
            else:
                x_ref_ramp['current'] = max(x_ref_ramp['current'] - Ramp_rate*dt, x_ref)
        else:
            x_ref_ramp['last_t'] = t  # keep last_t updated even when not ramping
        
        x_ref_vec = np.array([x_ref_ramp['current'], 0, 0, 0])

        # ----- controller selection -----
        if abs(theta_err) < angle_threshold and abs(theta_dot) < velocity_threshold:

            # LQR stabilization
            error = x_hat - x_ref_vec
            u = float(-K @ error)
        else:

            # swing-up
            u = float(swing_up_controller(state, params, k_swing))
            
        # actuator saturation
        #i am already doing it in swing up controller
        #u = np.clip(u, -20, 20)


        # ----- plant dynamics -----
        x_dot_vec = cartpole_nonlinear(t, state, u, params)


        # ----- observer update (ALWAYS RUN) -----
        innovation = y - y_pred

        # wrap innovation angle
        innovation[1] = ((innovation[1] + np.pi) % (2*np.pi)) - np.pi

        x_hat_dot = A @ x_hat + B.flatten()*u + L @ innovation
        
        
      #  if t < 0.1:
            #print(f"y={y}, y_pred={y_pred}, innovation={innovation}")
        
       
        return np.concatenate((x_dot_vec, x_hat_dot))


    # time grid
    t_eval = np.linspace(0, t_final, 2000)

    sol = solve_ivp(
        closed_loop,
        [0, t_final],
        Z0,
        t_eval=t_eval,
        max_step=0.01
    )

    return sol