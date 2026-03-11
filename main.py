import numpy as np
import matplotlib.pyplot as plt

from dynamics.linearized import linear_matrices
from simulation.simulate import simulate
from my_control.lqr import compute_lqr
from my_control.kalman import compute_kalman_gain
from utils.config import PARAMS, INITIAL_STATE, Q, R, T_FINAL
from utils.animation import animate_cartpole


def plot_states(sol):

    t = sol.t

    x = sol.y[0]
    x_dot = sol.y[1]
    theta = sol.y[2]
    theta_dot = sol.y[3]

    x_hat = sol.y[4]
    x_dot_hat = sol.y[5]
    theta_hat = sol.y[6]
    theta_dot_hat = sol.y[7]

    theta_deg = np.degrees(theta)
    theta_hat_deg = np.degrees(theta_hat + np.pi)

    fig, axs = plt.subplots(4,1,figsize=(8,10))

    axs[0].plot(t,x,label="x")
    axs[0].plot(t,x_hat,"--",label="x_hat")
    axs[0].set_ylabel("Cart Position (m)")
    axs[0].legend()

    axs[1].plot(t,x_dot,label="x_dot")
    axs[1].plot(t,x_dot_hat,"--",label="x_dot_hat")
    axs[1].set_ylabel("Cart Velocity (m/s)")
    axs[1].legend()

    axs[2].plot(t,theta_deg,label="theta")
    axs[2].plot(t,theta_hat_deg,"--",label="theta_hat")
    axs[2].axhline(180,linestyle=":",color="gray")
    axs[2].set_ylabel("Angle (deg)")
    axs[2].legend()

    axs[3].plot(t,theta_dot,label="theta_dot")
    axs[3].plot(t,theta_dot_hat,"--",label="theta_dot_hat")
    axs[3].set_ylabel("Angular Velocity (rad/s)")
    axs[3].set_xlabel("Time (s)")
    axs[3].legend()

    plt.tight_layout()


def main():

    A,B = linear_matrices(PARAMS)

    print("Eigenvalues of A:\n",np.linalg.eigvals(A))

    K = compute_lqr(A,B,Q,R)

    print("Closed-loop eigenvalues:\n",np.linalg.eigvals(A-B@K))

    C = np.array([
        [1,0,0,0],
        [0,0,1,0]
    ])

    Qk = np.diag([
    1e-2,   # x process noise
    1e-1,   # x_dot process noise  
    1e-2,   # theta process noise
    1e-1,   # theta_dot process noise — most uncertain, we differentiate it
    ])
    Rk = np.diag([
    1e-4,   # x measurement — encoder, very accurate
    1e-4,   # theta measurement — encoder, very accurate
    ])

    L = compute_kalman_gain(A,C,Qk,Rk)

    print("Kalman Gain:\n",L)

    print("\nEnter desired cart equilibrium position (meters)")
    x_ref = float(input("x_ref = "))

    sol = simulate(INITIAL_STATE, K, A, B, C, L, PARAMS, T_FINAL, x_ref)

    print("\nChoose display option:")
    print("1 → Plot states")
    print("2 → Animation")
    print("3 → Both")

    choice = input("Enter choice: ")

    if choice=="1":
        plot_states(sol)
        plt.show()

    elif choice=="2":
        anim = animate_cartpole(sol,PARAMS, x_ref)
        plt.show()

    elif choice=="3":
        plot_states(sol)
        anim = animate_cartpole(sol,PARAMS, x_ref)
        plt.show()

    else:
        print("Invalid option")


if __name__=="__main__":
    main()