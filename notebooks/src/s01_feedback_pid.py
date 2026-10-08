# %% [markdown]
# # Session 1 · Feedback and PID from first principles
#
# **Drone-control bootcamp at Space²** · notebook 1 of 7
#
# By the end of this notebook you can
#
# 1. simulate a first-order plant with a few lines of Python and check it against the exact solution;
# 2. predict the steady-state error and speed of a proportional controller *before* simulating it;
# 3. write a PID update that matches, number for number, the **easyPID** library you will flash on an Arduino in the lab;
# 4. choose PID gains for a second-order plant that meet a written specification.
#
# **How this notebook works.** Cells marked *Exercise* contain a `TODO`: replace the
# `raise NotImplementedError(...)` with your code. The cell after each exercise is an
# **auto-check**: run it, and it prints `[PASS]` or tells you what is wrong. The rest of
# the notebook never depends on your answers, so if you are stuck you can move on and
# come back.

# %%
import matplotlib.pyplot as plt
import numpy as np

from bootcamp import easypid as ep
from bootcamp.checks import check_cases, check_close, check_less, check_true, run
from bootcamp.loop import simulate_open_loop, simulate_pid
from bootcamp.plants import ARDUINO, first_order, rc_ladder, second_order
from bootcamp.plotting import REF, SERIES, save

# %% [markdown]
# ## 1. A plant, open loop
#
# Most things you will control in this bootcamp respond like a **first-order lag** when
# you look at them on the right time scale: a heater, a motor's speed, the voltage on a
# capacitor charged through a resistor (lab 1), a propeller's thrust after an ESC
# command. Its model is
#
# $$\tau\,\dot y(t) + y(t) = K\,u(t)$$
#
# with input $u$, output $y$, **static gain** $K$ (output per unit input once everything
# has settled) and **time constant** $\tau$ (how long it takes). For a step $u = u_0$
# starting from $y(0)=0$ the exact solution is
#
# $$y(t) = K u_0 \left(1 - e^{-t/\tau}\right),$$
#
# so after one time constant the output has covered $1 - e^{-1} \approx 63.2\,\%$ of the way.
#
# A computer cannot solve the differential equation in closed form for an arbitrary
# input, so it **steps** it: the simplest way is the *forward-Euler* step
#
# $$y_{k+1} = y_k + \Delta t\,\frac{K u_k - y_k}{\tau}.$$


# %% [exercise 1.1]
def euler_step(y, u, K, tau, dt):
    """One forward-Euler step of tau*y' + y = K*u. Returns y after dt seconds."""
    ### BEGIN SOLUTION
    return y + dt * (K * u - y) / tau
    ### END SOLUTION


# %% [check 1.1]
check_cases(
    "1.1",
    euler_step,
    [
        ((0.0, 1.0, 2.0, 0.5, 0.01), 0.04),
        ((1.0, 0.0, 2.0, 0.5, 0.1), 0.8),
        ((3.0, 2.0, 1.5, 1.0, 0.2), 3.0),
    ],
    hint="dy/dt = (K*u - y)/tau, so y_next = y + dt * dy/dt",
)
# Euler against the exact solution, for a coarse and a fine step
K, tau = 2.0, 0.5
fig, ax = plt.subplots(figsize=(7, 3.2))
t_fine = np.linspace(0, 3, 300)
ax.plot(t_fine, K * (1 - np.exp(-t_fine / tau)), color=SERIES[0], label="exact")
for dt, c in ((0.25, SERIES[1]), (0.05, SERIES[2])):
    ys = [0.0]
    for _ in range(int(3 / dt)):
        ys.append(run("1.1", euler_step, ys[-1], 1.0, K, tau, dt))
    ax.plot(np.arange(len(ys)) * dt, ys, "o-", ms=3, color=c, label=f"Euler, dt = {dt} s")
ax.set(xlabel="time (s)", ylabel="y", title="Forward Euler converges as dt shrinks")
_ = ax.legend()

# %% [markdown]
# The library simulates plants with the **exact** discretisation instead (it is still one
# line per step, see `bootcamp.plants.LinearPlant`), so from now on numerical error is not
# something we have to think about. Here is the plant of lab 1 -- a 10 kΩ resistor
# charging a 100 µF capacitor, $\tau = RC = 1$ s, $K = 1$ V/V -- driven open loop with
# 2.5 V:

# %%
rc = rc_ladder(10e3, 100e-6, stages=1)
t, y, u = simulate_open_loop(rc, 2.5, 5.0, 0.01)
fig, ax = plt.subplots(figsize=(7, 3.0))
ax.plot(t, y, color=SERIES[0], label="capacitor voltage")
ax.axhline(2.5, **REF, label="input 2.5 V")
ax.axvline(1.0, color=SERIES[3], lw=1, ls=":", label="t = RC = 1 s")
ax.set(
    xlabel="time (s)", ylabel="volts", title="Lab-1 plant, open loop: 63 % after one time constant"
)
ax.legend()
print(f"y(1 s) = {y[100]:.3f} V = {100 * y[100] / 2.5:.1f} % of the final value")

# %% [markdown]
# ## 2. Closing the loop with a proportional controller
#
# Open loop, $y$ only reaches the setpoint if we know $K$ exactly and nothing disturbs
# the plant. **Feedback** measures $y$ and acts on the error $e = r - y$. The simplest
# law is proportional, $u = K_p e$. Substituting into the plant:
#
# $$\tau \dot y + y = K K_p (r - y) \quad\Longrightarrow\quad \frac{\tau}{1 + K K_p}\,\dot y + y = \frac{K K_p}{1 + K K_p}\, r.$$
#
# The closed loop is again first order, with
#
# * a steady-state value $y_\infty = \dfrac{K K_p}{1 + K K_p}\, r$ -- *never quite* $r$: the
#   **steady-state error** $r - y_\infty = r / (1 + K K_p)$ shrinks with $K_p$ but never vanishes;
# * a time constant $\tau_{cl} = \dfrac{\tau}{1 + K K_p}$ -- feedback makes the plant faster.


# %% [exercise 1.2]
def p_control_prediction(K, tau, Kp, r):
    """Steady-state output and closed-loop time constant of tau*y' + y = K*u with u = Kp*(r - y).

    Returns (y_inf, tau_cl).
    """
    ### BEGIN SOLUTION
    L = K * Kp
    return L / (1 + L) * r, tau / (1 + L)
    ### END SOLUTION


# %% [check 1.2]
check_cases(
    "1.2",
    p_control_prediction,
    [((1.0, 1.0, 4.0, 1.0), (0.8, 0.2)), ((2.0, 0.5, 1.5, 10.0), (7.5, 0.125))],
    hint="loop gain L = K*Kp; y_inf = L/(1+L)*r and tau_cl = tau/(1+L)",
)
# ... and against a simulation with the easyPID port, P only
plant = first_order(2.0, 0.5)
res = simulate_pid(plant, ep.PIDController(1.5, 0.0, 0.0, -1e3, 1e3), 10.0, 3.0, 0.002)
y_inf, tau_cl = run("1.2", p_control_prediction, 2.0, 0.5, 1.5, 10.0)
check_close("1.2", res.y[-1], y_inf, rtol=1e-3, what="simulated steady state")
k = np.searchsorted(res.t, tau_cl)
check_close("1.2", res.y[k] / y_inf, 1 - np.exp(-1), atol=0.01, what="y(tau_cl)/y_inf")

# %% [markdown]
# Several gains side by side (the plant is the lab-1 RC circuit, setpoint 2.5 V, and the
# controller is the **easyPID** port, `bootcamp.easypid.PIDController`, used exactly like the
# C++ class: `update(setpoint, measurement, dt_in_ms)`):

# %%
fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
for kp, c in zip((0.5, 2.0, 8.0), SERIES):
    pid = ep.PIDController(kp, 0.0, 0.0, 0.0, 5.0)  # output limited to 0..5 V like the PWM pin
    r = simulate_pid(rc, pid, 2.5, 4.0, 0.01)
    axs[0].plot(r.t, r.y, color=c, label=f"Kp = {kp}")
    axs[1].plot(r.t, r.u, color=c, label=f"Kp = {kp}")
axs[0].axhline(2.5, **REF, label="setpoint")
axs[0].set(xlabel="time (s)", ylabel="y (V)", title="P control: faster, but always short")
axs[1].set(xlabel="time (s)", ylabel="u (V)", title="controller output (saturates at 5 V)")
axs[0].legend()
axs[1].legend()
save(fig, "s01_p_only")

# %% [markdown]
# Two things to notice. Higher $K_p$ gives a smaller offset and a faster response, as
# predicted. And with $K_p = 8$ the output starts *at the 5 V limit*: the actuator
# saturates, the loop is temporarily open, and the response is slower than the formula
# says. Saturation is the subject of session 2.
#
# ## 3. Integral action removes the offset
#
# The offset exists because a P controller needs a non-zero error to produce a non-zero
# output. An **integral** term accumulates the error, $u_I = K_i \int_0^t e\,dt$, and keeps
# growing as long as any error remains -- so in steady state the error must be zero.
# Adding the derivative term gives the full PID law
#
# $$u(t) = K_p\,e(t) + K_i \int_0^t e(\sigma)\,d\sigma + K_d\,\frac{de(t)}{dt}.$$
#
# A microcontroller runs it at discrete times $t_k$, $\Delta t$ apart. **easyPID** (Rami
# Kronbi's Arduino library, which you will flash in the lab) implements exactly this:
#
# | term | easyPID update, every call | note |
# |---|---|---|
# | P | $p_k = K_p e_k$ | |
# | I | $I_k = I_{k-1} + e_k \Delta t$, $\;i_k = K_i I_k$ | rectangular integration |
# | D | $d_k = K_d (e_k - e_{k-1})/\Delta t$ | **zero on the first update** after `begin()`/`reset()` (no "derivative kick") |
# | output | $u_k = \mathrm{clamp}(p_k + i_k + d_k,\, u_{min},\, u_{max})$ | |
#
# $\Delta t$ is in **seconds** in the formulas but you pass it to easyPID in
# **milliseconds**: `pid.update(setpoint, measurement, dtMs)`. easyPID also has
# anti-windup (on by default) -- we switch it off for this exercise and meet it in session 2.


# %% [exercise 1.3]
class MyPID:
    """A PID controller with easyPID's update rule (no anti-windup)."""

    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def update(self, setpoint, measurement, dt_ms):
        """Return the clamped output. dt_ms is in milliseconds."""
        ### BEGIN SOLUTION
        dt = dt_ms / 1000.0
        e = setpoint - measurement
        self.integral += e * dt
        if self.first:
            deriv = 0.0
            self.first = False
        else:
            deriv = (e - self.prev_error) / dt
        self.prev_error = e
        u = self.kp * e + self.ki * self.integral + self.kd * deriv
        return min(max(u, self.out_min), self.out_max)
        ### END SOLUTION


# %% [check 1.3]
rng = np.random.default_rng(1)
for trial in range(5):
    gains = rng.uniform(0, 3, 3)
    mine = run("1.3", MyPID, *gains, -20.0, 20.0)
    ref = ep.PIDController(*gains, -20.0, 20.0)
    ref.setAntiWindup(ep.ANTIWINDUP_NONE)
    ref.begin()
    for k in range(200):
        sp, y, dt = rng.uniform(-5, 5), rng.uniform(-5, 5), rng.integers(1, 40)
        check_close(
            "1.3",
            run("1.3", mine.update, sp, y, dt),
            ref.update(sp, y, dt),
            rtol=1e-9,
            atol=1e-9,
            what=f"output at step {k} of trial {trial}",
            hint="integral += e*dt (dt in seconds!), D is zero on the first call, clamp the sum",
            quiet=True,
        )
check_true("1.3", True, "MyPID matches easyPID (ANTIWINDUP_NONE) on 1000 random updates")

# %% [markdown]
# P, PI and PID on the RC circuit. With integral action the offset is gone; the price
# is a slower final approach (and, with too much $K_i$, overshoot):

# %%
fig, ax = plt.subplots(figsize=(7, 3.2))
for (kp, ki, kd), c, name in zip(
    ((2.0, 0, 0), (2.0, 2.0, 0), (2.0, 2.0, 0.1)), SERIES, ("P", "PI", "PID")
):
    r = simulate_pid(rc, ep.PIDController(kp, ki, kd, 0.0, 5.0), 2.5, 6.0, 0.01)
    ax.plot(r.t, r.y, color=c, label=f"{name}: Kp={kp}, Ki={ki}, Kd={kd}")
ax.axhline(2.5, **REF, label="setpoint")
ax.set(xlabel="time (s)", ylabel="y (V)", title="Integral action removes the steady-state error")
ax.legend()
save(fig, "s01_p_pi_pid")

# %% [markdown]
# ## 4. A second-order plant: why the D term exists
#
# A mass on a spring, a pendulum, a drone's attitude: anything with **inertia** stores
# energy in two forms and can oscillate. The standard model is
#
# $$\ddot y + 2\zeta\omega_n \dot y + \omega_n^2 y = K\omega_n^2 u,$$
#
# with natural frequency $\omega_n$ (rad/s) and damping ratio $\zeta$. For $\zeta < 1$ a
# step overshoots by $100\,e^{-\pi\zeta/\sqrt{1-\zeta^2}}$ %. Take $K=1$, $\omega_n = 2$,
# $\zeta = 0.2$: lightly damped, 53 % overshoot open loop.
#
# With **PD** control, $u = K_p e + K_d \dot e$, and a constant setpoint ($\dot e = -\dot y$):
#
# $$\ddot y + (2\zeta\omega_n + K\omega_n^2 K_d)\,\dot y + \omega_n^2 (1 + K K_p)\, y = K\omega_n^2 K_p r.$$
#
# So $K_p$ raises the stiffness and $K_d$ adds damping. The closed loop has
#
# $$\omega_{cl} = \omega_n\sqrt{1 + K K_p}, \qquad \zeta_{cl} = \frac{2\zeta\omega_n + K\omega_n^2 K_d}{2\,\omega_{cl}}.$$


# %% [exercise 1.4]
def pd_closed_loop(K, wn, zeta, Kp, Kd):
    """Natural frequency and damping ratio of the second-order plant under PD control.

    Returns (w_cl, zeta_cl).
    """
    ### BEGIN SOLUTION
    w_cl = wn * np.sqrt(1 + K * Kp)
    zeta_cl = (2 * zeta * wn + K * wn**2 * Kd) / (2 * w_cl)
    return w_cl, zeta_cl
    ### END SOLUTION


# %% [check 1.4]
def _eig_reference(K, wn, zeta, Kp, Kd):
    A = np.array([[0, 1], [-(wn**2) * (1 + K * Kp), -(2 * zeta * wn + K * wn**2 * Kd)]])
    lam = np.linalg.eigvals(A)
    w = np.sqrt(np.prod(lam).real)
    return w, -lam.sum().real / (2 * w)


cases = [
    ((1.0, 2.0, 0.2, 3.0, 0.5), _eig_reference(1.0, 2.0, 0.2, 3.0, 0.5)),
    ((2.0, 5.0, 0.05, 1.0, 0.1), _eig_reference(2.0, 5.0, 0.05, 1.0, 0.1)),
]
check_cases(
    "1.4",
    pd_closed_loop,
    cases,
    rtol=1e-6,
    hint="compare the closed-loop equation with y'' + 2 zeta_cl w_cl y' + w_cl^2 y",
)

# %%
plant2 = second_order(1.0, 2.0, 0.2)
fig, ax = plt.subplots(figsize=(7, 3.2))
for g, c, name in zip(((3.0, 0, 0), (3.0, 0, 0.6), (3.0, 3.0, 0.6)), SERIES, ("P", "PD", "PID")):
    r = simulate_pid(plant2, ep.PIDController(*g, -10, 10), 1.0, 6.0, 0.01)
    ax.plot(r.t, r.y, color=c, label=f"{name}: Kp, Ki, Kd = {g}")
ax.axhline(1.0, **REF, label="setpoint")
ax.set(
    xlabel="time (s)", ylabel="y", title="Second-order plant: D adds damping, I removes the offset"
)
ax.legend()
save(fig, "s01_second_order")

# %% [markdown]
# ## 5. Meet a specification
#
# Choose gains for the same plant ($K=1$, $\omega_n=2$, $\zeta=0.2$) such that, for a unit
# step, with easyPID sampling every 10 ms and the output limited to ±10:
#
# | | limit |
# |---|---|
# | overshoot | < 10 % |
# | settling time (2 % band) | < 2.5 s |
# | steady-state error | < 1 % |
#
# Use exercise 1.4 to pick $K_p$, $K_d$ for a well-damped loop, then add a little $K_i$.
# Try values in a scratch cell first; the check simulates your gains.


# %% [exercise 1.5]
def my_gains():
    """Return (kp, ki, kd) for the specification above."""
    ### BEGIN SOLUTION
    # Kp = 3 gives w_cl = 2*sqrt(4) = 4 rad/s; Kd = 1.5 gives zeta_cl = (0.8 + 6)/8 = 0.85
    return 3.0, 2.0, 1.5
    ### END SOLUTION


# %% [check 1.5]
g = run("1.5", my_gains)
r = simulate_pid(second_order(1.0, 2.0, 0.2), ep.PIDController(*g, -10, 10), 1.0, 8.0, 0.01)
info = r.step_info()
print(info)
check_less("1.5", info.overshoot_pct, 10.0, "overshoot", " %")
check_less(
    "1.5",
    info.settling_time,
    2.5,
    "settling time",
    " s",
    hint="more Kp for speed, more Kd for damping",
)
check_less("1.5", abs(info.steady_state_error), 0.01, "|steady-state error|", hint="Ki > 0")
fig, ax = plt.subplots(figsize=(7, 3.0))
ax.plot(r.t, r.y, color=SERIES[0], label=f"your gains {g}")
ax.axhline(1.0, **REF, label="setpoint")
ax.axhspan(0.98, 1.02, color=SERIES[0], alpha=0.12, label="2 % band")
ax.set(xlabel="time (s)", ylabel="y", title="Your controller")
_ = ax.legend()

# %% [markdown]
# ## 6. Before the lab: predict what you will measure
#
# In the lab you close the loop around the RC circuit with an Arduino. The sketch
# `arduino/s01_rc_plant` works in volts: it converts the 0-5 V command to a PWM duty
# cycle on pin 5 and averages four `analogRead`s (10-bit, about 4.9 mV per count). Here
# is what the model predicts for the gains you will try, *including* the ADC quantisation
# and the 0-5 V output limits. Keep this plot open during the lab and compare.


# %%
def adc(v):  # what analogRead sees, converted back to volts
    return ARDUINO.volts_to_adc(v) * 5.0 / 1023


def steps(t):  # the sketch's "steps 1.5 3.5 8" command
    return 1.5 if (t % 16) < 8 else 3.5


fig, ax = plt.subplots(figsize=(9, 3.4))
for g, c in zip(((1.0, 0.0, 0.0), (2.0, 1.0, 0.0), (4.0, 3.0, 0.0)), SERIES):
    r = simulate_pid(rc, ep.PIDController(*g, 0.0, 5.0), steps, 24.0, 0.01, measure=adc)
    ax.plot(r.t, r.y, color=c, label=f"Kp, Ki, Kd = {g}")
ax.plot(r.t, r.r, **REF, label="setpoint")
ax.set(xlabel="time (s)", ylabel="capacitor voltage (V)", title="Prediction for lab 1 (RC = 1 s)")
ax.legend(ncol=2)
save(fig, "s01_lab_prediction")

# %% [markdown]
# ### Reflection (not auto-checked)
#
# Your real capacitor is probably not exactly 100 µF (electrolytics are often ±20 %).
# Which features of the measured response tell you its real value, and how would you
# read $\tau$ off an open-loop step?
#
# <!-- BEGIN SOLUTION -->
# The time constant: $\tau = RC$, so measure the time to 63.2 % of the final value in an
# open-loop step (`open`, `u 2.5` from 0 V) and divide by $R$. The static gain stays 1 V/V
# whatever the capacitance, so a different $C$ changes only the speed of the response,
# not where it settles. In closed loop, the P-only offset does not depend on $C$ either;
# the time constants do.
# <!-- END SOLUTION -->
#
# **Summary.** Feedback trades a known model for a measurement. P control is fast but
# leaves an offset $r/(1+KK_p)$; I removes it; D damps plants that store energy. easyPID
# implements the discrete PID you just wrote, plus the safety features we add in session 2.
