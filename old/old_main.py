import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from simulation.simulate import simulate
from my_control.lqr import compute_lqr
from dynamics.linearized import linear_matrices
from config import INITIAL_STATE, T_FINAL, PARAMS, Q, R
from my_control import ctrb
def animate_cartpole(sol, params):

    x = sol.y[0]
    theta = sol.y[2]
    l = params['l']

    fig, ax = plt.subplots()
    fig.patch.set_facecolor("black")
    ax.set_facecolor("black")

    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect("equal")
    ax.axis("off")

    # Cart dimensions
    cart_width = 0.5
    cart_height = 0.25
    wheel_radius = 0.08

    # Cart body
    cart = plt.Rectangle((0, 0), cart_width, cart_height, color="grey")
    ax.add_patch(cart)

    # Wheels
    wheel_left = plt.Circle((0, 0), wheel_radius, color="grey")
    wheel_right = plt.Circle((0, 0), wheel_radius, color="grey")
    ax.add_patch(wheel_left)
    ax.add_patch(wheel_right)

    # Pendulum rod
    pole_line, = ax.plot([], [], lw=2, color="white")

    # Pendulum mass
    mass = plt.Circle((0, 0), 0.07, color="red")
    ax.add_patch(mass)

    def update(frame):

        cart_x = x[frame]
        angle = np.pi - theta[frame]  # Flip angle to match upright reference

        # Update cart body
        cart.set_xy((cart_x - cart_width / 2, -cart_height / 2))

        # Update wheels
        wheel_left.center = (
            cart_x - cart_width / 4,
            -cart_height / 2 - wheel_radius
        )

        wheel_right.center = (
            cart_x + cart_width / 4,
            -cart_height / 2 - wheel_radius
        )

        # Pendulum end position
        pole_x = cart_x + l * np.sin(angle)
        pole_y = l * np.cos(angle)

        pole_line.set_data([cart_x, pole_x], [0, pole_y])
        mass.center = (pole_x, pole_y)

        return cart, wheel_left, wheel_right, pole_line, mass

    anim = FuncAnimation(
        fig,
        update,
        frames=len(x),
        interval=20,
        blit=False
    )

    return anim


def main():

    A, B = linear_matrices(PARAMS)
    K = compute_lqr(A, B, Q, R)

    #check controllability
    rank_ctrb = ctrb.get_controllability_rank(A, B)
    print(f"Controllability matrix rank: {rank_ctrb})")



    print("Eigenvalues of A:\n", np.linalg.eigvals(A))
    print("B:\n", B)
    #print("INITIAL_STATE:\n", INITIAL_STATE)
    #print("LQR Gain Matrix K:\n", K)
    #print("Closed-loop eigenvalues:\n", np.linalg.eigvals(A - B @ K))

    sol = simulate(INITIAL_STATE, K, PARAMS, T_FINAL)

    anim = animate_cartpole(sol, PARAMS)
    plt.show()


if __name__ == "__main__":
    main()