# %% [markdown]
# # Session 5 · Quadrotor dynamics and control in simulation
#
# **Drone-control bootcamp at Space²** · notebook 5 of 7
#
# The rig is one axis. A quadrotor has six degrees of freedom and four motors, and the
# same ideas scale up: the motors produce a thrust and three torques through a **mixing
# (allocation) matrix**; a fast **rate** loop and an **attitude** loop keep it level; a
# slower **velocity** and **position** loop tell the attitude loop where to tilt. This
# notebook uses `bootcamp.quad`, a deliberately small simulator (about 300 lines -- read
# it) whose vehicle is the nominal 5-inch quad of Rami Kronbi's rotor-fault study, and
# whose PIDs are easyPID controllers.
#
# By the end you can
#
# 1. derive the allocation matrix of an X quadrotor and compute the hover point;
# 2. explain how a mixer gives up yaw first when the motors saturate;
# 3. map a desired acceleration to a tilt command;
# 4. tune a position loop to a specification and see time-scale separation at work;
# 5. say what a minimal model leaves out by comparing it with a research model.

# %%
import math

import matplotlib.pyplot as plt
import numpy as np

from bootcamp import quad
from bootcamp.checks import check_cases, check_close, check_less, check_true, run
from bootcamp.metrics import step_info
from bootcamp.plotting import REF, SERIES, save

P = quad.QuadParams()
print(
    f"mass {P.mass} kg, arm {P.arm} m, max thrust {P.f_max:.2f} N per rotor, thrust/weight {P.thrust_to_weight:.2f}"
)

# %% [markdown]
# ## 1. Rigid-body dynamics in one screen
#
# With world frame z up, body frame FLU (x forward, y left, z up), attitude $R$ (body to
# world) and body rates $\omega$:
#
# $$m\dot v = R\begin{bmatrix}0\\0\\T\end{bmatrix} - R D R^T v - m g\,e_3, \qquad J\dot\omega = \tau - \omega\times J\omega - K_\omega\,\omega,$$
#
# where $T = \sum f_i$ is the collective thrust, $D$ a linear drag, $K_\omega$ a rotational
# damping. Each rotor makes thrust $f_i = k_f\Omega_i^2$ and a reaction torque
# $s_i\kappa f_i$ about z (the sign $s_i$ depends on its spin direction); its speed follows
# the command with a lag $\tau_m = 25$ ms.
#
# ## 2. The allocation matrix
#
# Rotors are numbered from above: 1 front-right, 2 front-left, 3 rear-left, 4 rear-right,
# at positions $(x_i, y_i)$ (azimuths -45°, 45°, 135°, 225°). Rotor $i$ adds $y_i f_i$ to the
# roll torque and $-x_i f_i$ to the pitch torque (thrust up, z up: check it with the
# right-hand rule). So
#
# $$\begin{bmatrix}T\\\tau_x\\\tau_y\\\tau_z\end{bmatrix} = B f, \qquad B = \begin{bmatrix}1 & 1 & 1 & 1\\ y_1 & y_2 & y_3 & y_4\\ -x_1 & -x_2 & -x_3 & -x_4\\ \kappa s_1 & \kappa s_2 & \kappa s_3 & \kappa s_4\end{bmatrix}.$$


# %% [exercise 5.1]
def mixer_matrix(positions, spin_dir, kappa):
    """B (4x4) from rotor positions (4x3, body frame), spin directions s_i and kappa."""
    ### BEGIN SOLUTION
    r = np.asarray(positions, dtype=float)
    return np.vstack([np.ones(4), r[:, 1], -r[:, 0], kappa * np.asarray(spin_dir, dtype=float)])
    ### END SOLUTION


# %% [check 5.1]
B = run("5.1", mixer_matrix, P.rotor_positions, P.spin_dir, P.kappa)
check_close("5.1", B, P.mixer(), atol=1e-12, what="B")
try:
    from rotorfault.params import nominal_params

    check_close(
        "5.1", B, nominal_params().mixer(), atol=1e-12, what="B against the rotorfault study model"
    )
except ImportError:
    print("(rotorfault not installed: skipped the comparison with the study model)")
print(np.round(B, 4))

# %% [markdown]
# **Hover.** In hover all torques are zero and $T = mg$. Because $B$ is invertible, the
# only solution is $f_i = mg/4$ for every rotor, i.e. $\Omega_h = \sqrt{mg/(4k_f)}$. The
# thrust-to-weight ratio $4 f_{max}/(mg)$ is the margin left for manoeuvring (and, in
# session 6, for surviving a fault).


# %% [exercise 5.2]
def hover_rotor_speed(mass, g, k_f):
    """Rotor speed (rad/s) at hover."""
    ### BEGIN SOLUTION
    return math.sqrt(mass * g / (4 * k_f))
    ### END SOLUTION


# %% [check 5.2]
w_h = run("5.2", hover_rotor_speed, P.mass, P.g, P.k_f)
check_close("5.2", w_h, quad.Quadrotor(P).x[13], rtol=1e-12, what="hover speed (rad/s)")
print(
    f"hover: {w_h:.0f} rad/s = {w_h * 60 / (2 * math.pi):.0f} rpm, {100 * P.hover_thrust / P.f_max:.0f} % of max thrust"
)

# %% [markdown]
# Without feedback, hover is not stable. Make rotor 2 just 1 % stronger than the rest:

# %%
q = quad.Quadrotor(P)
f = np.full(4, P.hover_thrust) * np.array([1.0, 1.01, 1.0, 1.0])
rolls = []
for _ in range(1500):
    q.step(f, 0.001)
    rolls.append(math.degrees(quad.R_to_euler_zyx(q.R)[0]))
print(f"after 1.5 s open loop: roll {rolls[-1]:.0f} deg, position {np.round(q.p, 2)} m")

# %% [markdown]
# ## 3. Mixing with saturation: yaw goes first
#
# Solving $f = B^{-1}[T, \tau]$ can ask for more than a motor can give. Something must be
# sacrificed, in order of importance for staying in the air:
#
# 1. **roll/pitch** torque (attitude) -- keep if at all possible;
# 2. **collective thrust** -- shift it by the smallest amount that fits;
# 3. **yaw** torque -- a quad's yaw authority is small ($\kappa = 1.5$ cm vs an arm of 11 cm),
#    and spinning slowly is harmless: scale it down first.
#
# For the last step: with $f_0$ the rotor thrusts without yaw and $d = B^{-1}[0,0,0,\tau_z]$
# the yaw part, find the largest $\alpha\in[0,1]$ such that $f_{min} \le f_0 + \alpha d \le f_{max}$
# for every rotor.


# %% [exercise 5.3]
def yaw_scale(f0, d, f_min, f_max):
    """Largest alpha in [0, 1] with f_min <= f0 + alpha*d <= f_max element-wise."""
    ### BEGIN SOLUTION
    alpha = 1.0
    for fi, di in zip(f0, d):
        if di > 0:
            alpha = min(alpha, (f_max - fi) / di)
        elif di < 0:
            alpha = min(alpha, (f_min - fi) / di)
    return max(alpha, 0.0)
    ### END SOLUTION


# %% [check 5.3]
check_cases(
    "5.3",
    yaw_scale,
    [
        ((np.full(4, 1.5), np.array([0.1, -0.1, 0.1, -0.1]), 0.05, 5.6), 1.0),
        ((np.full(4, 5.4), np.array([0.4, -0.4, 0.4, -0.4]), 0.05, 5.6), 0.5),
        ((np.array([5.0, 0.3, 5.0, 0.3]), np.array([1.0, -1.0, 1.0, -1.0]), 0.05, 5.6), 0.25),
        ((np.full(4, 5.6), np.array([0.2, -0.2, 0.2, -0.2]), 0.05, 5.6), 0.0),
    ],
    hint="each rotor gives an upper bound on alpha; take the smallest, clipped to [0, 1]",
)

# %% [markdown]
# ## 4. From a desired acceleration to a tilt
#
# The position and velocity loops output a desired acceleration $a$. A quad can only push
# along its body z axis, so to accelerate horizontally it must tilt. For small angles the
# thrust direction is $R e_3 \approx (\theta\cos\psi + \phi\sin\psi,\ \theta\sin\psi - \phi\cos\psi,\ 1)$,
# so with thrust $\approx mg$:
#
# $$\theta_d = \frac{a_x\cos\psi + a_y\sin\psi}{g}, \qquad \phi_d = \frac{a_x\sin\psi - a_y\cos\psi}{g}.$$
#
# (A positive pitch tips the nose *down* in this z-up convention and accelerates forward.)


# %% [exercise 5.4]
def my_accel_to_tilt(ax, ay, yaw, g=9.81):
    """Small-angle (roll, pitch) in rad for a desired horizontal acceleration."""
    ### BEGIN SOLUTION
    c, s = math.cos(yaw), math.sin(yaw)
    return (ax * s - ay * c) / g, (ax * c + ay * s) / g
    ### END SOLUTION


# %% [check 5.4]
cases = [
    ((ax_, ay_, yaw), quad.accel_to_tilt(ax_, ay_, yaw))
    for ax_, ay_, yaw in ((1, 0, 0), (0, 1, 0), (1.5, -0.5, 0.7), (-2, 1, -2.5))
]
check_cases(
    "5.4",
    my_accel_to_tilt,
    cases,
    hint="pitch from the forward component, roll from minus the left component (yaw = 0)",
)

# %% [markdown]
# ## 5. The full cascade
#
# `quad.CascadedController` chains everything (all PIDs are easyPID controllers, with
# `CLAMP` anti-windup and an EMA filter on the rate D term):
#
# | loop | input -> output | default gains |
# |---|---|---|
# | position | error (m) -> velocity setpoint (m/s), limited to 3 m/s | P = 1.5 /s |
# | velocity | error (m/s) -> acceleration (m/s²) | P = 3, I = 1 |
# | attitude | angle error (rad) -> body-rate setpoint (rad/s) | P = 8 /s (roll, pitch) |
# | rate | rate error (rad/s) -> angular acceleration (rad/s²), times J -> torque | P = 25, I = 10, D = 0.25 |
#
# Each loop is several times faster than the one around it (**time-scale separation**), so
# each outer loop can treat the inner one as "it does what I ask, quickly".

# %%
log = quad.simulate_quad(
    quad.CascadedController(), quad.waypoints([(0, 0, 1), (1, 0, 1)], 1.0), 5.0
)
fig, axs = plt.subplots(1, 3, figsize=(12, 3.2))
axs[0].plot(log.t, log.p[:, 0], color=SERIES[0], label="x")
axs[0].plot(log.t, log.p_ref[:, 0], **REF, label="reference")
axs[0].set(xlabel="time (s)", ylabel="m", title="Position: 1 m step")
axs[1].plot(log.t, np.degrees(log.euler[:, 1]), color=SERIES[1], label="pitch")
axs[1].set(xlabel="time (s)", ylabel="deg", title="Attitude: tilt to accelerate, then brake")
axs[2].plot(log.t, log.rate_sp[:, 1], color=SERIES[3], label="rate setpoint (pitch)")
axs[2].plot(log.t, log.w[:, 1], color=SERIES[2], ls="--", label="body rate q")
axs[2].set(xlabel="time (s)", ylabel="rad/s", title="Rate loop tracks its setpoint closely")
for a in axs:
    a.legend()
save(fig, "s05_step")

# %% [markdown]
# ## 6. Tune the position loop
#
# Specification for a 1 m step in x (from hover at 1 m height):
#
# | | limit |
# |---|---|
# | overshoot | < 5 % |
# | settling into ±5 cm | < 1.5 s |
# | peak tilt | < 30° |
#
# The defaults miss it slightly. Change only the **outer** loops (`pos_kp`, `vel_kp`,
# `vel_ki`, `vel_kd`) and return a `quad.CascadeGains`. A hint worth thinking about: the
# velocity loop's integrator integrates *velocity* error, i.e. position error.


# %% [exercise 5.5]
def my_cascade_gains():
    """Return a quad.CascadeGains that meets the specification."""
    ### BEGIN SOLUTION
    # The horizontal velocity integrator only adds overshoot here (there is no steady
    # horizontal force to cancel); keep it on z, where it holds altitude against model error.
    return quad.CascadeGains(vel_ki=(0.0, 0.0, 2.0))
    ### END SOLUTION


# %% [check 5.5]
gains = run("5.5", my_cascade_gains)
log5 = quad.simulate_quad(
    quad.CascadedController(gains=gains), quad.waypoints([(0, 0, 1), (1, 0, 1)], 1.0), 5.0
)
check_true("5.5", not log5.crashed, "the vehicle did not crash")
m = log5.t >= 1.0
info = step_info(log5.t[m], log5.p[m, 0], 1.0, y0=0.0, band=0.05)
check_less("5.5", info.overshoot_pct, 5.0, "overshoot", " %")
check_less("5.5", info.settling_time, 1.5, "settling into +-5 cm", " s")
check_less(
    "5.5",
    math.degrees(np.abs(log5.euler[:, :2]).max()),
    30.0,
    "peak tilt",
    " deg",
    hint="lower pos_kp or the velocity limit",
)

# %% [markdown]
# ## 7. How much motor lag can the cascade take?
#
# The controller assumes nothing about the motors, but they sit inside the fastest loop.
# Multiply the true motor time constant (25 ms) and fly the same step with the default gains:

# %%
fig, ax = plt.subplots(figsize=(8, 3.2))
for scale, c in zip((1, 6, 8, 12), SERIES):
    lg = quad.simulate_quad(
        quad.CascadedController(),
        quad.waypoints([(0, 0, 1), (1, 0, 1)], 1.0),
        5.0,
        motor_scale=scale,
    )
    label = f"motor lag {25 * scale} ms" + (" (crashed)" if lg.crashed else "")
    ax.plot(lg.t, np.degrees(lg.euler[:, 1]), color=c, label=label)
ax.set(
    xlabel="time (s)",
    ylabel="pitch (deg)",
    title="Same gains, slower motors: the rate loop runs out of phase margin",
)
ax.set_ylim(-90, 90)
ax.legend()
save(fig, "s05_motor_lag")

# %% [markdown]
# Two motors' worth of lag is fine; a factor of eight makes the rate loop oscillate and
# twelve crashes it. The rotorfault study found the same kind of limit for its fault-tolerant
# controller (it failed at a motor time constant of 80 ms) -- the first thing to check on a
# real vehicle is how slow its actuators really are.
#
# ## 8. A minimal model against a research model
#
# Is a 300-line simulator good enough? Drive it and the rotorfault study model with the
# same rotor commands, open loop, for 0.4 s: first a pure roll command, then a pure yaw
# command.

# %%
try:
    from rotorfault.dynamics import QuadDynamics, hover_state, thrust_to_speed
    from rotorfault.params import nominal_params

    RP = nominal_params()
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.2))
    for ax, f_cmd, idx, name in (
        (axs[0], np.array([1.55, 1.58, 1.58, 1.55]), 10, "roll rate p"),
        (axs[1], np.array([1.62, 1.50, 1.62, 1.50]), 12, "yaw rate r"),
    ):
        x = hover_state(RP, (0, 0, 1.0))
        mq = quad.Quadrotor(P, (0, 0, 1.0))
        a, b = [], []
        for _ in range(400):
            x = QuadDynamics(RP).step(x, thrust_to_speed(f_cmd, RP), np.ones(4), 0.001)
            mq.step(f_cmd, 0.001)
            a.append(x[idx])
            b.append(mq.x[idx])
        tt = np.arange(400) * 0.001
        ax.plot(tt, a, color=SERIES[0], label="rotorfault study model")
        ax.plot(tt, b, color=SERIES[1], ls="--", label="bootcamp minimal model")
        ax.set(xlabel="time (s)", ylabel="rad/s", title=name)
        ax.legend()
    save(fig, "s05_model_check")
except ImportError:
    print("rotorfault not installed: see docs/setup.md")

# %% [markdown]
# ### Reflection (not auto-checked)
#
# The roll responses are identical; the yaw responses are not. Read the docstring of
# `rotorfault.dynamics` (or the session notes): which term does the minimal model leave
# out, and why does it matter for yaw but not for roll?
#
# <!-- BEGIN SOLUTION -->
# The reaction torque of **accelerating rotors**, $s_i J_r \dot\Omega_i$ about z (plus the
# rotor gyroscopic term $\omega \times h_r$). A yaw command speeds two rotors up and slows
# two down; while their speeds change, the change of rotor angular momentum reacts on the
# body about z and adds to the drag torque $\kappa f$, which is itself small. A roll command
# (left pair vs right pair) changes rotors of *both* spin directions equally, so their
# reaction torques cancel. For yaw the transient term is comparable to the steady drag
# torque, so the minimal model under-predicts the initial yaw acceleration.
# <!-- END SOLUTION -->
#
# **Summary.** $B$ turns four thrusts into thrust and three torques; invert it, and
# saturate yaw first. Tilt to accelerate. Nest the loops with each several times faster
# than the next, and know your actuator delay: it sets how fast the innermost loop may be.
