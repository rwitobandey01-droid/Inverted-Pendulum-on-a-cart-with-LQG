# Cart-Pole Swing-Up and Stabilization with LQR + Kalman Observer

![Python](https://img.shields.io/badge/Python-3.10+-blue)
![Status](https://img.shields.io/badge/Status-Simulation%20Complete-green)
![Next](https://img.shields.io/badge/Next-ROS2%20%2B%20Gazebo-orange)

A full nonlinear simulation of a cart-pole system implementing energy-based swing-up control, LQR stabilization, and a Kalman observer — built entirely from scratch in Python.

---

## Demo

> Pendulum swings up from rest, hands off to LQR, stabilizes at target cart position.

![demo](assets/demo.gif)

---

## Theory
<img width="1920" height="1080" alt="Screenshot from 2026-03-12 02-57-19" src="https://github.com/user-attachments/assets/53187e25-937e-46ae-bdc0-b5ec48b6fa96" />

### System

The cart-pole consists of a cart of mass $M$ on a with friction track, with a pendulum of mass $m$ and length $l$ pivoting freely at the top. The state vector is:

$$\mathbf{x} = [x,\ \dot{x},\ \theta,\ \dot{\theta}]$$

where $x$ is cart position and $\theta$ is pendulum angle. The convention used here is:

- $\theta = 0$ → hanging down
- $\theta = \pi$ → upright (control target)

### Nonlinear Dynamics

The full equations of motion are derived from the Euler-Lagrange equations:

$$\ddot{x} = \frac{(I + ml^2)(u - b\dot{x}) + m^2l^2\dot{\theta}^2\sin\theta - m^2gl^2\sin\theta\cos\theta}{D}$$

$$\ddot{\theta} = \frac{-ml(u - b\dot{x})\cos\theta - ml(M+m)g\sin\theta + m^2l^2\dot{\theta}^2\sin\theta\cos\theta}{D}$$

where $D = (I(M+m) + Mml^2) - (ml\cos\theta)^2$.

### Linearization

The system is linearized around the upright equilibrium $\theta = \pi$, substituting $\phi = \theta - \pi$ (so $\phi = 0$ at upright). This yields the standard $A$, $B$ matrices used for LQR and observer design.

### Swing-Up (Energy Shaping)

Since LQR is only valid near the upright position, an energy-based swing-up controller is used to bring the pendulum from rest to near-upright:

$$E = \frac{1}{2}(I + ml^2)\dot{\theta}^2 + mgl(1 - \cos\theta)$$

$$u_{swing} = k_{swing}(E - E_{des}) \cdot \text{sign}(\dot{\theta}\cos\theta)$$

The desired energy $E_{des} = 2mgl$ corresponds to the pendulum standing upright with zero velocity. Setting $E_{des} = 0.95 \times 2mgl$ causes the pendulum to arrive just under upright speed, making the LQR handoff smoother.

### LQR Stabilization

Once the pendulum is near upright, an LQR controller minimizes:

$$J = \int_0^\infty (\mathbf{e}^T Q \mathbf{e} + u^T R u)\ dt$$

where $\mathbf{e} = \hat{\mathbf{x}} - \mathbf{x}_{ref}$ is the error between the estimated state and reference. The gain matrix $K$ is computed by solving the continuous algebraic Riccati equation (CARE).

### Kalman Observer

Since not all states are directly measurable (only $x$ and $\theta$ via encoders), a Luenberger observer driven by Kalman gain estimates the full state:

$$\dot{\hat{\mathbf{x}}} = A\hat{\mathbf{x}} + Bu + L(\mathbf{y} - C\hat{\mathbf{x}})$$

The observer gain $L$ is computed as the dual of the LQR problem using process noise covariance $Q_k$ and measurement noise covariance $R_k$.

### Controller Switching

A threshold-based switching law transitions between swing-up and LQR:

```
if |θ_err| < angle_threshold AND |θ_dot| < velocity_threshold:
    use LQR
else:
    use swing-up
```

### Reference Tracking with Ramp

To move the cart to an arbitrary position $x_{ref}$ without destabilizing the pendulum, the reference is ramped linearly rather than applied as a step:

$$x_{ref}(t) = x_{ref}(t - dt) + r \cdot dt$$

where $r$ is the ramp rate in m/s. This prevents the large instantaneous force that a step reference would demand.

---

## Project Structure

```
cart_pendulum_lqr/
│
├── dynamics/
│   ├── nonlinear.py        # Full nonlinear equations of motion
│   └── linearized.py       # Linearized A, B matrices around upright
│
├── my_control/
│   ├── lqr.py              # LQR gain computation (CARE solver)
│   ├── swing_up.py         # Energy-shaping swing-up controller
│   └── ctrb.py             # Controllability check
│
├── simulation/
│   └── simulate.py         # Main simulation loop (solve_ivp)
│
├── utils/
│   ├── config.py           # All parameters, gains, thresholds
│   └── kalman.py           # Kalman observer gain computation
│
├── plotting.py             # State plots
├── animation.py            # Matplotlib cart-pole animation
├── main.py                 # Entry point
└── README.md
```

---

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| $M$ | 1.0 kg | Cart mass |
| $m$ | 0.1 kg | Pendulum mass |
| $l$ | 0.5 m | Pendulum length |
| $g$ | 9.81 m/s² | Gravity |
| $I$ | 1/3·ml² | Pendulum moment of inertia |
| $b$ | 0.1 | Cart damping coefficient |

### Tuned Gains

```python
Q = diag([100, 1, 300, 5])   # state cost: prioritize angle and position
R = diag([0.7])               # control cost: moderate effort
k_swing = 6                   # swing-up energy gain
angle_threshold = 1.0 rad     # LQR entry angle threshold
velocity_threshold = 8 rad/s  # LQR entry velocity threshold
Ramp_rate = 1.5 m/s           # reference ramp rate
force_lim = 20 N              # actuator saturation limit
```

---

## Hurdles Encountered

This project was significantly harder in practice than in theory. Here are the real challenges faced during development:

### 1. Coordinate Frame Mismatch
The observer was initialized in the linearized frame ($\theta - \pi$) but measurements were fed in the raw frame ($\theta$). This caused a persistent ~$\pi$ offset in the innovation signal, making the observer diverge slowly and eventually blow up at around t=27s with states reaching magnitudes of $10^{52}$.

**Fix:** Shift measurements to the linear frame before computing innovation: `y[1] = theta - pi`.

### 2. Kalman Noise Ratio Wrong
Initial Kalman tuning had $Q_k \ll R_k$, telling the filter to trust the model over measurements. The observer barely corrected from innovations, causing a persistent 0.15 rad angle offset that prevented LQR from ever stabilizing.

**Fix:** Flip the ratio — large $Q_k$, small $R_k$ — so the observer aggressively corrects from the accurate encoder measurements.

### 3. Swing-Up Stuck at Bottom
The stuck-kick heuristic used `|theta_dot| < 1e-3` to detect a stalled pendulum and applied a tiny 0.2N kick. Since `force_lim = 20N`, this was only 1% of available force — completely insufficient to start the swing. The pendulum would rock ±1° at the bottom forever.

**Fix:** Detect low energy instead of low velocity, and use a much stronger kick: `if E < 0.1 * E_des: return 5.0 * sign(cos(theta))`.

### 4. LQR Never Activating
Debug prints revealed `theta_err = 179.2°` for the entire 30 second simulation — the pendulum was oscillating at the bottom, never reaching upright. LQR requires `theta_err < 57°` to activate, so it never fired once.

**Fix:** Combined stronger kick + correct energy calculation using full rotational inertia `(I + ml²)` instead of just `ml²`.

### 5. LQR Chattering at Handoff
Once swing-up was fixed, LQR would activate for 2-3 timesteps then immediately deactivate because `theta_dot` briefly spiked above `velocity_threshold` as the solver evaluated intermediate RK45 stages.

**Fix:** Hysteresis switching — easy entry condition (`|theta_err| < 57°` AND `|theta_dot| < 8`), hard exit condition (`|theta_err| > 40°` only). Uses a mutable dict to persist state across `solve_ivp` calls.

### 6. Reference Tracking Failing for Arbitrary Positions
The controller worked for x_ref ≈ 10m (where the cart happened to land during swing-up) but failed for x_ref = 1m or x_ref = 20m. The root cause was that a step reference created a large instantaneous position error at LQR handoff, generating a large force that tipped the pendulum before LQR could stabilize it.

**Fix:** Ramp the reference at 1.5 m/s using time-based integration (`dt = t - last_t`). Note: naive ramp without tracking `dt` still fails because `solve_ivp` (RK45) calls the function ~600 times per second, making the ramp effectively instantaneous.

### 7. RK45 Call Frequency Misunderstanding
The ramp was initially implemented as `x_ref += ramp_rate` per function call. Since RK45 calls the dynamics function 6+ times per step for error estimation and adaptive stepsize control, this moved the reference ~72 m/s instead of 1.5 m/s.

**Fix:** Track real elapsed time: `x_ref += ramp_rate * (t - last_t)`.

---

## Results

| Test | Result |
|------|--------|
| Swing-up from rest | ✅ ~15 seconds |
| Stabilization at upright | ✅ θ_err < 0.1° in steady state |
| x_ref = 0m | ✅ |
| x_ref = 10m | ✅ |
| x_ref = 20m | ✅ (with t_final = 50s) |
| Observer convergence | ✅ tracks within swing-up transient |

---

## Installation

```bash
git clone https://github.com/yourusername/cart_pendulum_lqr
cd cart_pendulum_lqr
pip install numpy scipy matplotlib
python main.py
```

---

## Usage

```bash
python main.py
# Enter desired cart equilibrium position (meters): 10
# Choose display option:
# 1 → Plot states
# 2 → Animation
# 3 → Both
```

---

## Roadmap

- [x] Nonlinear simulation
- [x] Energy-based swing-up
- [x] LQR stabilization
- [x] Kalman observer
- [x] Hysteresis switching
- [x] Arbitrary reference tracking with ramp
- [ ] **ROS2 integration** — wrap controller as a ROS2 node, publish/subscribe to joint states and force commands
- [ ] **Gazebo simulation** — replace Python dynamics with a physics-accurate Gazebo model, test controller on simulated hardware
- [ ] **Real hardware** — deploy on physical cart-pole with encoder feedback
- [ ] **MPC** — replace LQR with Model Predictive Control for constraint handling

---

## References

- Åström & Furuta (2000) — *Swinging up a pendulum by energy control*
- Ogata — *Modern Control Engineering*
- Franklin, Powell & Emami-Naeini — *Feedback Control of Dynamic Systems*


https://github.com/user-attachments/assets/966454bd-162c-438b-a9cf-957102cb1ce7

