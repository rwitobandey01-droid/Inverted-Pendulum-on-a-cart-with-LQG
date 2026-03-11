import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle, FancyArrow
from matplotlib.lines import Line2D


def animate_cartpole(sol, params, x_ref=None):

    x = sol.y[0]
    theta = sol.y[2]
    l = params["l"]

    # --- figure setup ---
    fig, ax = plt.subplots(figsize=(20, 10))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")

    # fixed world view — cart moves within it
    track_half = 15  # show 15m either side
    ax.set_xlim(-track_half, track_half)
    ax.set_ylim(-0.8, 1.5)
    ax.set_aspect("equal")
    ax.axis("off")

    # --- track / ground ---
    # ground line
    ax.axhline(y=-0.2, color="#555577", linewidth=2, zorder=1)

    # track rails
    ax.axhline(y=-0.12, color="#888899", linewidth=3, zorder=2)

    # tick marks on track (every 1m)
    for tx in range(-track_half, track_half + 1):
        ax.plot([tx, tx], [-0.20, -0.12], color="#555577", lw=1, zorder=2)
        ax.text(tx, -0.30, f"{tx}m", color="#888899",
                fontsize=6, ha="center", va="top")

    # origin marker
    ax.plot([0, 0], [-0.20, -0.08], color="#ffff00",
            lw=2, zorder=3, linestyle="--", alpha=0.5)
    ax.text(0, -0.38, "origin", color="#ffff00",
            fontsize=7, ha="center", alpha=0.7)

    # --- x_ref marker ---
    if x_ref is not None:
        ax.plot([x_ref, x_ref], [-0.20, 0.8], color="#00ff88",
                lw=1.5, zorder=3, linestyle="--", alpha=0.7)
        ax.text(x_ref, 0.85, f"ref={x_ref}m", color="#00ff88",
                fontsize=8, ha="center", alpha=0.9)

    # --- cart ---
    cart_width = 0.5
    cart_height = 0.22
    wheel_radius = 0.06

    cart = Rectangle(
        (0, 0), cart_width, cart_height,
        color="#268ee4", zorder=5, linewidth=1.5,
        edgecolor="#094430"
    )
    ax.add_patch(cart)

    wheel1 = Circle((0, 0), wheel_radius, color="#dddddd",
                    zorder=6, linewidth=1, edgecolor="#aaaaaa")
    wheel2 = Circle((0, 0), wheel_radius, color="#dddddd",
                    zorder=6, linewidth=1, edgecolor="#aaaaaa")
    ax.add_patch(wheel1)
    ax.add_patch(wheel2)

    # wheel spokes (visual detail)
    spoke1, = ax.plot([], [], color="#aaaaaa", lw=1, zorder=7)
    spoke2, = ax.plot([], [], color="#aaaaaa", lw=1, zorder=7)

    # --- pole ---
    line, = ax.plot([], [], lw=3, color="#f5a623", zorder=8)

    # --- pendulum mass ---
    mass = Circle((0, 0), 0.07, color="#e74c3c",
                  zorder=9, linewidth=1.5, edgecolor="#ff6b6b")
    ax.add_patch(mass)

    # --- pivot dot ---
    pivot = Circle((0, 0), 0.04, color="#ffffff", zorder=10)
    ax.add_patch(pivot)

    # --- time display ---
    time_text = ax.text(
        -track_half + 0.5, 1.35, "", color="white",
        fontsize=10, fontweight="bold", zorder=11
    )

    # --- position display ---
    pos_text = ax.text(
        -track_half + 0.5, 1.15, "", color="#4a90d9",
        fontsize=9, zorder=11
    )

    # --- angle display ---
    angle_text = ax.text(
        -track_half + 0.5, 0.95, "", color="#f5a623",
        fontsize=9, zorder=11
    )

    # --- velocity trail (last N positions) ---
    trail_len = 30
    trail, = ax.plot([], [], color="#4a90d9",
                     alpha=0.3, lw=2, zorder=3)
    trail_x = []
    trail_y = []

    def init():
        line.set_data([], [])
        trail.set_data([], [])
        spoke1.set_data([], [])
        spoke2.set_data([], [])
        return cart, line, mass, wheel1, wheel2, pivot, trail, time_text, pos_text, angle_text

    def update(frame):
        cart_x = x[frame]
        th = theta[frame]
        t = sol.t[frame]

        # --- cart body ---
        cart.set_xy((cart_x - cart_width/2, -cart_height/2))

        # --- wheels ---
        w1x = cart_x - 0.15
        w2x = cart_x + 0.15
        wy = -cart_height/2 - wheel_radius + 0.02
        wheel1.center = (w1x, wy)
        wheel2.center = (w2x, wy)

        # rotating spoke effect
        angle = frame * 0.3
        for spoke, wx in [(spoke1, w1x), (spoke2, w2x)]:
            spoke.set_data(
                [wx - wheel_radius * np.cos(angle),
                 wx + wheel_radius * np.cos(angle)],
                [wy - wheel_radius * np.sin(angle),
                 wy + wheel_radius * np.sin(angle)]
            )

        # --- pivot point ---
        pivot_y = cart_height / 2 - 0.05
        pivot.center = (cart_x, pivot_y)

        # --- pole and mass ---
        pole_x = cart_x + l * np.sin(th)
        pole_y = pivot_y - l * np.cos(th)

        line.set_data([cart_x, pole_x], [pivot_y, pole_y])
        mass.center = (pole_x, pole_y)

        # --- cart trail ---
        trail_x.append(cart_x)
        trail_y.append(-cart_height/2)
        if len(trail_x) > trail_len:
            trail_x.pop(0)
            trail_y.pop(0)
        trail.set_data(trail_x, trail_y)

        # --- text updates ---
        time_text.set_text(f"t = {t:.2f}s")
        pos_text.set_text(f"x = {cart_x:.2f}m")

        # normalize angle to degrees from upright
        theta_deg = np.degrees(((th - np.pi + np.pi) % (2*np.pi)) - np.pi)
        angle_text.set_text(f"θ error = {theta_deg:.1f}°")

        return (cart, line, mass, wheel1, wheel2, pivot,
                trail, spoke1, spoke2,
                time_text, pos_text, angle_text)

    anim = FuncAnimation(
        fig,
        update,
        frames=len(sol.t),
        init_func=init,
        interval=20,
        blit=False
    )

    return anim