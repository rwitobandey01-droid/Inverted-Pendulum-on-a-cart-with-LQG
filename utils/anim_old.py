import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle


def animate_cartpole(sol, params, x_ref=0):

    x = sol.y[0]
    theta = sol.y[2]

    l = params["l"]

    view_range = 3   # show +-3 meters around x_ref

    fig, ax = plt.subplots()
    fig.patch.set_facecolor("black")
    ax.set_facecolor("black")

    ax.set_xlim(x_ref - view_range, x_ref + view_range)
    ax.set_ylim(-1.5, 1.5)

    ax.set_aspect("equal")

    # Cart dimensions
    cart_width = 0.4
    cart_height = 0.2
    wheel_radius = 0.05

    cart = Rectangle(
        (-cart_width/2, -cart_height/2),
        cart_width,
        cart_height,
        color="grey"
    )
    ax.add_patch(cart)

    wheel1 = Circle((-0.15, -0.15), wheel_radius, color="white")
    wheel2 = Circle((0.15, -0.15), wheel_radius, color="white")

    ax.add_patch(wheel1)
    ax.add_patch(wheel2)

    line, = ax.plot([], [], lw=2, color="white")

    mass = Circle((0, 0), 0.06, color="red")
    ax.add_patch(mass)

    def init():
        line.set_data([], [])
        return cart, line, mass, wheel1, wheel2

    def update(frame):

        cart_x = x[frame]
        th = theta[frame]

        # cart position
        cart.set_xy((cart_x - cart_width/2, -cart_height/2))

        wheel1.center = (cart_x - 0.15, -0.15)
        wheel2.center = (cart_x + 0.15, -0.15)

        pole_x = cart_x + l * np.sin(th)
        pole_y = -l * np.cos(th)

        line.set_data([cart_x, pole_x], [0, pole_y])
        mass.center = (pole_x, pole_y)

        return cart, line, mass, wheel1, wheel2

    anim = FuncAnimation(
        fig,
        update,
        frames=len(sol.t),
        init_func=init,
        interval=20,
        blit=True
    )

    return anim