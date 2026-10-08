# %% [markdown]
# # Session 3 · The seesaw rig: one axis of a drone on the bench
#
# **Drone-control bootcamp at Space²** · notebook 3 of 7
#
# Tonight you build and fly the rig: a beam on a pivot, a motor and propeller at each end,
# a potentiometer on the pivot axle that measures the angle. Push one motor up and the
# other down and the beam rotates -- exactly how a quadrotor rolls. Before any propeller
# spins, this notebook answers the questions that keep fingers safe and the first test
# boring: *what will the rig do, which gains are safe to start with, and what must the
# software refuse to do?*
#
# By the end you can
#
# 1. turn a kitchen-scale measurement into a thrust curve and a torque gain;
# 2. find the rig's hidden centre-of-gravity offset from its free-swing period;
# 3. explain, with closed-loop poles, why proportional control alone fails on this rig;
# 4. write the mixer and the arming logic that the sketch uses;
# 5. choose PID gains that meet a specification in simulation -- and know why that is
#    only a starting point for the hardware.
#
# All model parameters are **assumptions** for the build in `hardware/rig` until you
# replace them with your measurements (section 1 and 2).

# %%
import math

import matplotlib.pyplot as plt
import numpy as np

from bootcamp import easypid as ep
from bootcamp import rig
from bootcamp.checks import check_cases, check_close, check_less, run
from bootcamp.metrics import step_info
from bootcamp.plotting import REF, SERIES, save

P = rig.RigParams()
print(P)

# %% [markdown]
# ## 1. The rig's model
#
# With $\theta$ the beam angle (positive when the motor-1 end is up), $l$ the arm length,
# $J$ the inertia about the pivot, $m$ the moving mass, $h$ the distance of its centre of
# gravity **below** the pivot and $b$ a viscous friction:
#
# $$J\ddot\theta = l\,(T_1 - T_2) - m g h \sin\theta - b\dot\theta .$$
#
# The motors are commanded with ESC pulses of 1000-2000 µs. Throttle $x = (\text{us} - 1000)/1000$,
# and the static thrust of a propeller grows roughly with the square of its speed, so
#
# $$T(x) = c_1 x + c_2 x^2 ,$$
#
# reached through a first-order lag $\tau_m$ (motor and propeller inertia, ~80 ms for a
# 10-inch prop). The mixer turns one controller output $u$ (µs) into two commands,
# $\text{us}_1 = \text{base} + u$, $\text{us}_2 = \text{base} - u$.
#
# ### Measuring thrust with a kitchen scale
#
# In lab 3 you rest the motor-2 end of the beam on a scale, tare it, and spin only motor
# 1. With equal arms, the moment balance about the pivot says the reading rises by
# exactly $T_1$. The `s03_esc_bench` sketch steps the pulse from 1100 to 1600 µs. Below
# is an **example data set generated from the model with noise** -- replace it with
# yours (`data/thrust_template.csv` is the sheet to fill in).

# %%
us_meas = np.arange(1100, 1601, 50)
rng = np.random.default_rng(3)
grams_meas = P.thrust(us_meas) / 9.81 * 1000 + rng.normal(0, 4, len(us_meas))  # +-4 g scale noise
for u_, g_ in zip(us_meas, grams_meas):
    print(f"{u_} us: {g_:6.1f} g")


# %% [exercise 3.1]
def fit_thrust_curve(us, grams):
    """Least-squares fit of T = c1*x + c2*x^2 (newtons), x = (us - 1000)/1000. Returns (c1, c2)."""
    ### BEGIN SOLUTION
    x = (np.asarray(us, dtype=float) - 1000.0) / 1000.0
    T = np.asarray(grams, dtype=float) / 1000.0 * 9.81
    A = np.column_stack([x, x**2])
    c, *_ = np.linalg.lstsq(A, T, rcond=None)
    return float(c[0]), float(c[1])
    ### END SOLUTION


# %% [check 3.1]
x = (us_meas - 1000) / 1000
c_ref, *_ = np.linalg.lstsq(np.column_stack([x, x**2]), grams_meas / 1000 * 9.81, rcond=None)
c1, c2 = run("3.1", fit_thrust_curve, us_meas, grams_meas)
check_close(
    "3.1",
    (c1, c2),
    c_ref,
    rtol=1e-6,
    what="(c1, c2)",
    hint="grams -> newtons: /1000 * 9.81; no constant term",
)
fig, ax = plt.subplots(figsize=(6, 3.0))
ax.plot(us_meas, grams_meas, "o", color=SERIES[0], label="scale readings")
uu = np.linspace(1000, 1650, 100)
xx = (uu - 1000) / 1000
ax.plot(uu, (c1 * xx + c2 * xx**2) / 9.81 * 1000, color=SERIES[1], label="fit c1 x + c2 x^2")
ax.set(
    xlabel="ESC pulse (us)", ylabel="thrust (g)", title="Thrust curve of one motor (example data)"
)
ax.legend()
save(fig, "s03_thrust_curve")

# %% [markdown]
# Around the base command the rig is linear: a small differential $u$ changes the
# torque by
#
# $$\tau = 2\,l\,\frac{dT}{d\,\text{us}}\Big|_{\text{base}}\,u, \qquad \frac{dT}{d\,\text{us}} = \frac{c_1 + 2c_2x_0}{1000},$$
#
# with $x_0$ the base throttle. This **torque gain** (N·m per µs) is the plant gain
# every controller design on this rig starts from.


# %% [exercise 3.2]
def torque_gain(c1, c2, arm, base_us):
    """N*m of torque per microsecond of differential command around base_us."""
    ### BEGIN SOLUTION
    x0 = (base_us - 1000.0) / 1000.0
    return 2 * arm * (c1 + 2 * c2 * x0) / 1000.0
    ### END SOLUTION


# %% [check 3.2]
A, B = rig.linear_model(P, 1350.0)
k_ref = B[2] * P.tau_motor  # the steady-state torque per us in the linear model
check_close(
    "3.2",
    run("3.2", torque_gain, P.thrust_max * P.thrust_a1, P.thrust_max * P.thrust_a2, P.arm, 1350.0),
    k_ref,
    rtol=1e-9,
    what="torque gain (N m/us)",
)
k_u = torque_gain(P.thrust_max * P.thrust_a1, P.thrust_max * P.thrust_a2, P.arm, 1350.0)
print(
    f"100 us of differential -> {100 * k_u:.3f} N m -> {100 * k_u / P.inertia:.0f} rad/s^2 of angular acceleration"
)

# %% [markdown]
# ## 2. The free swing tells you where the centre of gravity is
#
# With the motors idle the rig is a pendulum: for small angles
# $J\ddot\theta = -mgh\,\theta$, so it swings with period $T = 2\pi\sqrt{J/(mgh)}$. You know
# $m$ (kitchen scale) and can compute $J$ from the geometry; time ten swings with a phone
# and you get $h$ -- a number that is very hard to measure directly and that decides how
# the rig behaves: $h > 0$ is a pendulum (stable), $h = 0$ is neutral like a drone in
# hover, $h < 0$ falls over.


# %% [exercise 3.3]
def cg_offset_from_period(period_s, inertia, mass, g=9.81):
    """Distance of the centre of gravity below the pivot (m) from the free-swing period."""
    ### BEGIN SOLUTION
    return inertia * (2 * math.pi / period_s) ** 2 / (mass * g)
    ### END SOLUTION


# %% [check 3.3]
log = rig.simulate_rig(
    lambda t, sp, m, dt: 0.0,
    15.0,
    theta0_deg=6.0,
    params=rig.with_changes(P, damping=0.002),
    dt=0.002,
)
up = np.nonzero((log.theta[:-1] < 0) & (log.theta[1:] >= 0))[0]
period = float(np.diff(log.t[up]).mean())
h = run("3.3", cg_offset_from_period, period, P.inertia, P.mass)
check_close("3.3", h, P.cg_below_pivot, rtol=0.02, what="h from the simulated swing (m)")
print(f"swing period {period:.2f} s -> h = {1000 * h:.1f} mm")

# %% [markdown]
# ## 3. Why P alone fails here
#
# Close the loop with P only, $u = -K_p\theta$ (gain in µs per degree). In the linear model
# (states $\theta$, $\dot\theta$, motor torque) the closed-loop poles are the eigenvalues
# of $A - B K_p \frac{180}{\pi} [1, 0, 0]$. A pendulum with weak damping plus an 80 ms
# motor lag: watch the poles cross into the right half-plane.

# %%
deg = 180 / math.pi
kps = np.linspace(0.05, 4, 200)
re_p = [np.linalg.eigvals(A - np.outer(B, [kp * deg, 0, 0])).real.max() for kp in kps]
re_pd = [np.linalg.eigvals(A - np.outer(B, [kp * deg, 0.3 * deg, 0])).real.max() for kp in kps]
fig, ax = plt.subplots(figsize=(7, 3.2))
ax.plot(kps, re_p, color=SERIES[1], label="P only")
ax.plot(kps, re_pd, color=SERIES[0], label="PD, Kd = 0.3 us/(deg/s)")
ax.axhline(0, **REF, label="stability boundary")
ax.set(
    xlabel="Kp (us per degree)",
    ylabel="max Re(pole) (1/s)",
    title="Rig closed-loop poles: P alone goes unstable",
)
ax.legend()
save(fig, "s03_p_only_instability")
kp_crit = kps[np.argmax(np.array(re_p) > 0)]
print(
    f"P-only becomes unstable at Kp = {kp_crit:.2f} us/deg; with Kd = 0.3 it stays stable over the whole range"
)

# %% [markdown]
# That is why the lab sketch starts at `kp 0.5 kd 0.3` and why you **never** try P-only
# with a large gain on the hardware: in simulation it bangs the beam into the stops.
# The derivative term needs a rate signal; here it comes from differencing the pot angle,
# in session 4 from a gyro.
#
# ## 4. The mixer
#
# Two motors, one command. If `base + u` would exceed the throttle cap, simply clipping
# would reduce the *difference* between the motors, i.e. the torque, exactly when the
# controller asks for more. Instead shift both commands (the "air mode" idea of
# Betaflight): keep $\text{us}_1 - \text{us}_2 = 2u$ and move the base. Only when $2|u|$
# exceeds the whole range must the differential itself be limited to $\pm(\text{hi}-\text{lo})/2$.


# %% [exercise 3.4]
def my_mix(base, u, lo, hi):
    """Return (us1, us2) = (base + u, base - u), shifted (or u limited) to stay in [lo, hi]."""
    ### BEGIN SOLUTION
    u = max(-(hi - lo) / 2, min((hi - lo) / 2, u))
    m1, m2 = base + u, base - u
    top, bot = max(m1, m2), min(m1, m2)
    shift = hi - top if top > hi else (lo - bot if bot < lo else 0.0)
    return m1 + shift, m2 + shift
    ### END SOLUTION


# %% [check 3.4]
cases = [
    ((b, u, 1100.0, 1600.0), rig.mix(b, u, 1100.0, 1600.0))
    for b, u in ((1350, 50), (1350, -120), (1550, 100), (1150, -80), (1350, 400), (1350, -900))
]
check_cases("3.4", my_mix, cases, hint="keep m1 - m2 = 2u; shift both by the overflow")

# %% [markdown]
# ## 5. Arming: what the software must refuse
#
# The physical safety chain comes first (e-stop in the 12 V line, guards, clamp, nobody in
# the prop plane -- `hardware/rig/safety.md`). The software adds interlocks. The sketch
# (`arduino/common/rig_safety.h`) arms **only** if *all* of these hold:
#
# * the arm switch is ON **and** has been seen OFF since boot or the last disarm (a switch
#   left on at power-up must not start the motors);
# * the pot reading is plausible: 10 ≤ counts ≤ 1013 (a loose wiper wire reads 0 or 1023);
# * the beam is within ±20° of level;
# * at least 3000 ms have passed since boot (the ESCs have finished their start-up beeps).


# %% [exercise 3.5]
def my_arming_allowed(switch_on, switch_was_off, pot_counts, angle_deg, ms_since_boot):
    """True only when every arming rule above holds."""
    ### BEGIN SOLUTION
    return (
        switch_on
        and switch_was_off
        and 10 <= pot_counts <= 1013
        and abs(angle_deg) <= 20.0
        and ms_since_boot >= 3000
    )
    ### END SOLUTION


# %% [check 3.5]
grid = [
    (sw, was, pot, ang, ms)
    for sw in (True, False)
    for was in (True, False)
    for pot in (0, 10, 512, 1013, 1023)
    for ang in (-25.0, -20.0, 3.0, 20.0, 21.0)
    for ms in (2999, 3000, 60000)
]
cases = [(args, float(rig.arming_allowed(*args))) for args in grid]


def arming_as_number(*args):
    return float(run("3.5", my_arming_allowed, *args))


check_cases("3.5", arming_as_number, cases, hint="all five conditions, boundaries included")

# %% [markdown]
# ## 6. Tune the single loop in simulation
#
# Specification for a 0 → 10° setpoint step (the `seq` command does this on the rig),
# with the pot quantisation, motor lag and the 1100-1600 µs limits of the sketch:
#
# | | limit |
# |---|---|
# | overshoot | < 15 % |
# | settling into ±1° | < 2.0 s |
# | never closer than 2° to the ±30° stops | |
# | **also** stable and < 30 % overshoot if the centre of gravity sits exactly on the pivot ($h = 0$) | |
#
# The last line is robustness: you will not know $h$ exactly on the night. The sketch uses
# easyPID with `FILTER_EMA, 0.6` on the derivative; so does the check.


# %% [exercise 3.6]
def rig_gains():
    """Return (kp, ki, kd) in us/deg, us/(deg s), us/(deg/s)."""
    ### BEGIN SOLUTION
    # PD first (Kd = 0.8 for damping), then Ki for the slow tail of the pendulum
    return 2.0, 1.5, 0.8
    ### END SOLUTION


# %% [check 3.6]
def rig_step(gains, params, seed=0):
    pid = ep.PIDController(*gains, -200, 200)
    pid.setDerivativeFilter(ep.FILTER_EMA, 0.6)
    pid.begin()
    return rig.simulate_rig(
        lambda t, sp, m, dt: pid.update(sp, m["pot"], dt * 1000),
        6.0,
        setpoint=lambda t: 10.0 if t >= 1.0 else 0.0,
        params=params,
        seed=seed,
    )


g = run("3.6", rig_gains)
nominal = rig_step(g, P)
m = nominal.t >= 1.0
info = step_info(nominal.t[m], nominal.theta[m], 10.0, y0=0.0)
out = np.nonzero(np.abs(nominal.theta[m] - 10.0) > 1.0)[0]
settle_1deg = (
    nominal.t[m][out[-1] + 1] - 1.0
    if len(out) and out[-1] + 1 < m.sum()
    else (0.0 if not len(out) else math.inf)
)
check_less("3.6", info.overshoot_pct, 15.0, "overshoot (nominal rig)", " %")
check_less(
    "3.6",
    settle_1deg,
    2.0,
    "settling into +-1 deg",
    " s",
    hint="more Kd damps, more Ki removes the slow tail",
)
check_less("3.6", np.abs(nominal.theta).max(), 28.0, "largest |angle|", " deg")
neutral = rig_step(g, rig.with_changes(P, cg_below_pivot=0.0))
info0 = step_info(neutral.t[m], neutral.theta[m], 10.0, y0=0.0)
check_less(
    "3.6",
    info0.overshoot_pct,
    30.0,
    "overshoot with h = 0",
    " %",
    hint="a neutral rig needs more damping",
)
check_less("3.6", abs(neutral.theta[-100:].mean() - 10.0), 1.0, "final error with h = 0", " deg")
fig, ax = plt.subplots(figsize=(8, 3.2))
ax.plot(nominal.t, nominal.theta, color=SERIES[0], label=f"nominal rig, gains {g}")
ax.plot(neutral.t, neutral.theta, color=SERIES[1], ls="--", label="same gains, CG on the pivot")
ax.plot(nominal.t, nominal.setpoint, **REF, label="setpoint")
ax.set(xlabel="time (s)", ylabel="beam angle (deg)", title="Predicted step response on the rig")
_ = ax.legend()

# %% [markdown]
# ## 7. What the motors do
#
# A run with gains that meet the specification (Kp, Ki, Kd = 2, 1.5, 0.8), seen from the
# actuator's side. Note the base throttle (1350 µs) and the differential: the controller
# needs only tens of microseconds to hold 10°, because the CG offset is small. Check this
# on the night: if your rig needs much more, $h$ is larger than you think, or one motor is
# weaker (session 6).

# %%
pid = ep.PIDController(2.0, 1.5, 0.8, -200, 200)
pid.setDerivativeFilter(ep.FILTER_EMA, 0.6)
pid.begin()
demo = rig.simulate_rig(
    lambda t, sp, m, dt: pid.update(sp, m["pot"], dt * 1000),
    6.0,
    setpoint=lambda t: 10.0 if t >= 1.0 else 0.0,
)
fig, axs = plt.subplots(1, 2, figsize=(10, 3.2))
axs[0].plot(demo.t, demo.theta, color=SERIES[0], label="beam angle")
axs[0].plot(demo.t, demo.setpoint, **REF, label="setpoint")
axs[0].set(xlabel="time (s)", ylabel="deg", title="Gains 2 / 1.5 / 0.8 on the rig model")
axs[1].plot(demo.t, demo.us1, color=SERIES[0], label="motor 1")
axs[1].plot(demo.t, demo.us2, color=SERIES[1], label="motor 2")
axs[1].set(xlabel="time (s)", ylabel="ESC pulse (us)", title="Mixer output during the step")
axs[0].legend()
axs[1].legend()
save(fig, "s03_rig_step")

# %% [markdown]
# ### Reflection (not auto-checked)
#
# The simulation says your gains work. Give two reasons why the first run on the
# hardware should still start with *half* of them.
#
# <!-- BEGIN SOLUTION -->
# (1) Every parameter is an assumption until measured: thrust curve, motor lag, inertia,
# friction and the CG offset can each be off by tens of percent, and the motor lag in
# particular moves the stability limit. (2) The model has no vibration, no ESC
# non-linearity at low throttle, no flexing beam and no pot backlash; all of these add
# phase lag or noise, which the D term amplifies. Halving the gains keeps the first test
# inside the stable region while you measure; then increase in steps of ~25 %.
# <!-- END SOLUTION -->
#
# **Summary.** The rig is a drone axis: torque from differential thrust, a lag in every
# motor, and P alone is not enough. Measure the thrust curve and the swing period, design
# in simulation, then start the hardware gently with every interlock in place.
