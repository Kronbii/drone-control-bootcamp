# %% [markdown]
# # Session 2 · Tuning and real-world PID
#
# **Drone-control bootcamp at Space²** · notebook 2 of 7
#
# The PID of session 1 works on paper. On hardware it meets five problems: it runs at a
# finite **sample rate**, its actuator **saturates**, its sensor is **noisy**, the
# **setpoint jumps**, and nobody tells you the plant's parameters. easyPID has a feature for
# each of these; this notebook explains why each exists and how to tune with it.
#
# By the end you can
#
# 1. discretise a plant exactly and explain what the sample period costs you;
# 2. recognise integrator windup and pick an easyPID anti-windup mode;
# 3. choose the derivative filter coefficient from a cut-off frequency;
# 4. measure rise time, overshoot and settling time from a log;
# 5. identify a plant from a step test and tune it three ways: reaction curve, relay autotune
#    (easyPID's `PIDTuner`), and by hand.

# %%
import math

import matplotlib.pyplot as plt
import numpy as np

from bootcamp import easypid as ep
from bootcamp.checks import check_cases, check_close, check_true, run
from bootcamp.loop import simulate_open_loop, simulate_pid
from bootcamp.metrics import fit_fopdt_two_point, step_info, ziegler_nichols_open_loop
from bootcamp.plants import first_order, rc_ladder, second_order
from bootcamp.plotting import REF, SERIES, save

# %% [markdown]
# ## 1. Discretisation: what the computer actually controls
#
# The controller only sees the plant every $T_s$ seconds and holds its output constant in
# between (a **zero-order hold**). For the first-order plant $\tau\dot y + y = Ku$ with $u$
# constant over a sample, the exact solution over one period is
#
# $$y_{k+1} = a\,y_k + b\,u_k, \qquad a = e^{-T_s/\tau}, \quad b = K(1 - a).$$
#
# No approximation: this is what `bootcamp.plants` does for every plant (via the matrix
# exponential), and what the `s01_sim_plant` sketch does on the Arduino.


# %% [exercise 2.1]
def zoh_first_order(K, tau, Ts):
    """Exact ZOH discretisation of tau*y' + y = K*u. Returns (a, b)."""
    ### BEGIN SOLUTION
    a = math.exp(-Ts / tau)
    return a, K * (1 - a)
    ### END SOLUTION


# %% [check 2.1]
cases = []
for K, tau, Ts in ((1.0, 1.0, 0.01), (2.5, 0.3, 0.05), (0.7, 2.0, 0.5)):
    Ad, Bd = first_order(K, tau).discretise(Ts)
    cases.append(((K, tau, Ts), (Ad[0, 0], Bd[0])))
check_cases("2.1", zoh_first_order, cases, hint="a = exp(-Ts/tau), b = K*(1 - a)")

# %% [markdown]
# **The cost of sampling.** Holding the output for one period delays it by about $T_s/2$
# on average. A delay $T_d$ adds a phase lag of $360\,f\,T_d$ degrees at frequency $f$
# without changing the gain -- and phase lag is what turns a damped loop into an
# oscillating one. The same PID gains on the same plant, sampled more and more slowly:

# %%
plant = second_order(1.0, 2.0, 0.2)
fig, ax = plt.subplots(figsize=(7, 3.2))
for Ts, c in zip((0.005, 0.05, 0.15), SERIES):
    r = simulate_pid(plant, ep.PIDController(3.0, 2.0, 1.5, -10, 10), 1.0, 6.0, Ts)
    ax.step(r.t, r.y, where="post", color=c, label=f"Ts = {1000 * Ts:.0f} ms")
ax.axhline(1.0, **REF, label="setpoint")
ax.set(xlabel="time (s)", ylabel="y", title="Same gains, slower sampling: less damping")
ax.legend()
save(fig, "s02_sample_time")


# %% [exercise 2.2]
def hold_phase_lag_deg(f_hz, Ts):
    """Phase lag (degrees) at f_hz caused by the half-sample delay of a zero-order hold."""
    ### BEGIN SOLUTION
    return 360.0 * f_hz * Ts / 2
    ### END SOLUTION


# %% [check 2.2]
check_cases(
    "2.2",
    hold_phase_lag_deg,
    [((1.0, 0.1), 18.0), ((5.0, 0.004), 3.6), ((0.5, 0.15), 13.5)],
    hint="lag = 360 * f * delay, with delay = Ts/2",
)
print("Rule of thumb: sample at least 10-20 times faster than the closed-loop bandwidth.")
print(
    f"A 2 Hz loop sampled at 50 Hz loses {hold_phase_lag_deg(2.0, 0.02):.1f} deg of phase margin."
)

# %% [markdown]
# ## 2. Saturation and integrator windup
#
# A PWM pin cannot go beyond 5 V, an ESC beyond full throttle. While the output is
# **saturated** the loop is effectively open, but a naive integral term keeps integrating
# the error. When the error finally changes sign, the integrator is huge and has to
# "unwind" before the output leaves the limit: a large, slow overshoot. That is **windup**.
#
# easyPID has three modes (`setAntiWindup`):
#
# * `ANTIWINDUP_NONE` -- no protection (only for experiments like this one);
# * `ANTIWINDUP_CLAMP` (**the default**) -- *conditional integration*: if this sample's
#   output is saturated **and** the error pushes further into the limit, this sample's
#   integration is undone; integration that relieves the saturation is always allowed;
# * `ANTIWINDUP_BACKCALC` -- back-calculation: the excess $u_{clamped} - u_{raw}$ is fed
#   back into the integrator through $1/K_i$.

# %%
plant = rc_ladder(10e3, 100e-6, 1)
fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
sp = lambda t: 4.0 if t < 6 else 2.0  # noqa: E731 - 4 V needs more than the 3 V limit
for mode, c, name in zip(
    (ep.ANTIWINDUP_NONE, ep.ANTIWINDUP_CLAMP, ep.ANTIWINDUP_BACKCALC),
    SERIES,
    ("NONE", "CLAMP", "BACKCALC"),
):
    pid = ep.PIDController(2.0, 2.0, 0.0, 0.0, 3.0)
    pid.setAntiWindup(mode)
    r = simulate_pid(plant, pid, sp, 12.0, 0.01)
    axs[0].plot(r.t, r.y, color=c, label=name)
    axs[1].plot(r.t, r.i, color=c, label=name)
axs[0].plot(r.t, r.r, **REF, label="setpoint")
axs[0].set(xlabel="time (s)", ylabel="y (V)", title="Output limited to 3 V: setpoint 4 V, then 2 V")
axs[1].set(xlabel="time (s)", ylabel="I term (V)", title="What the integrator does meanwhile")
axs[0].legend()
axs[1].legend()
save(fig, "s02_windup")

# %% [markdown]
# With `NONE` the I term grows for 6 s and the output stays pinned at 3 V for seconds
# after the setpoint has dropped to 2 V. Implement easyPID's `CLAMP` rule yourself, for a
# PI controller (no D term), and the check compares it with the library on random data.


# %% [exercise 2.3]
def clamp_integration(integral_before, error, dt, kp, ki, out_min, out_max):
    """One step of easyPID's ANTIWINDUP_CLAMP for a PI controller.

    Accumulate (integral += error*dt), compute the raw output kp*e + ki*integral, and undo
    the accumulation if the raw output is above out_max with error > 0, or below out_min
    with error < 0. Return the new integral.
    """
    ### BEGIN SOLUTION
    integral = integral_before + error * dt
    raw = kp * error + ki * integral
    if (raw > out_max and error > 0) or (raw < out_min and error < 0):
        integral = integral_before
    return integral
    ### END SOLUTION


# %% [check 2.3]
rng = np.random.default_rng(2)
kp, ki = 1.5, 3.0
ref = ep.PIDController(kp, ki, 0.0, -2.0, 2.0)
ref.begin()
integral = 0.0
for k in range(400):
    sp, y, dt_ms = rng.uniform(-3, 3), rng.uniform(-3, 3), int(rng.integers(5, 50))
    ref.update(sp, y, dt_ms)
    integral = run("2.3", clamp_integration, integral, sp - y, dt_ms / 1000, kp, ki, -2.0, 2.0)
    check_close(
        "2.3",
        integral,
        ref.getIterm() / ki,
        rtol=1e-9,
        atol=1e-12,
        what=f"integral after update {k}",
        quiet=True,
        hint="restore integral_before only when pushing FURTHER into saturation",
    )
check_true("2.3", True, "clamp_integration matches easyPID's ANTIWINDUP_CLAMP on 400 updates")

# %% [markdown]
# ## 3. Derivative noise and the EMA filter
#
# The D term differentiates the error, and differentiating amplifies noise: a 1-count
# ADC flicker every 10 ms looks like a rate of 100 counts per second. easyPID can low-pass
# the derivative with an exponential moving average, `setDerivativeFilter(FILTER_EMA, alpha)`:
#
# $$d^{f}_k = \alpha\,d^{f}_{k-1} + (1 - \alpha)\,d_k .$$
#
# This is a first-order low-pass filter. Matching its decay $\alpha^k$ to $e^{-kT_s/\tau_f}$
# gives the time constant and cut-off frequency
#
# $$\tau_f = -\frac{T_s}{\ln\alpha}, \qquad f_c = \frac{1}{2\pi\tau_f} = -\frac{\ln\alpha}{2\pi T_s}, \qquad \alpha = e^{-2\pi f_c T_s}.$$
#
# So the same `alpha` means a *different* filter at a different sample rate.


# %% [exercise 2.4]
def ema_cutoff_hz(alpha, Ts):
    """Cut-off frequency (Hz) of easyPID's EMA derivative filter at sample period Ts (s)."""
    ### BEGIN SOLUTION
    return -math.log(alpha) / (2 * math.pi * Ts)
    ### END SOLUTION


def ema_alpha_for_cutoff(fc_hz, Ts):
    """The alpha that gives a cut-off of fc_hz at sample period Ts."""
    ### BEGIN SOLUTION
    return math.exp(-2 * math.pi * fc_hz * Ts)
    ### END SOLUTION


# %% [check 2.4]
check_cases("2.4", ema_cutoff_hz, [((0.7, 0.01), 5.6766), ((0.9, 0.005), 3.3536)], rtol=1e-4)
check_cases(
    "2.4", ema_alpha_for_cutoff, [((5.6766, 0.01), 0.7), ((10.0, 0.002), 0.88191)], rtol=1e-4
)
print(
    f"alpha = 0.7 at 100 Hz -> fc = {ema_cutoff_hz(0.7, 0.01):.1f} Hz; at 200 Hz -> {ema_cutoff_hz(0.7, 0.005):.1f} Hz"
)

# %%
plant = second_order(1.0, 2.0, 0.2)
fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
for alpha, c, name in zip(
    (None, 0.6, 0.9), SERIES, ("no filter", "EMA alpha 0.6", "EMA alpha 0.9")
):
    pid = ep.PIDController(3.0, 2.0, 1.5, -10, 10)
    if alpha is not None:
        pid.setDerivativeFilter(ep.FILTER_EMA, alpha)
    r = simulate_pid(plant, pid, 1.0, 4.0, 0.01, noise_std=0.01, seed=4)
    axs[0].plot(r.t, r.u, color=c, lw=1.0, label=name)
    axs[1].plot(r.t, r.y, color=c, label=name)
axs[0].set(xlabel="time (s)", ylabel="controller output", title="1 % sensor noise, Kd = 1.5")
axs[1].axhline(1.0, **REF, label="setpoint")
axs[1].set(xlabel="time (s)", ylabel="y", title="The plant barely notices; the actuator does")
axs[0].legend()
axs[1].legend()
save(fig, "s02_derivative_filter")

# %% [markdown]
# ## 4. Setpoint jumps and the derivative kick
#
# easyPID differentiates the **error**. When the setpoint jumps, the error jumps, and the D
# term produces one huge sample, $K_d\,\Delta r / T_s$ (easyPID suppresses this only on
# the very first update after `begin()`/`reset()`). On a drone that spike goes straight to
# the motors. The simple fix from easyPID's tuning guide is to **ramp the setpoint**:
# limit how far it may move per sample.


# %% [exercise 2.5]
def ramp_setpoint(current, target, max_step):
    """Move current towards target by at most max_step (> 0). Returns the new setpoint."""
    ### BEGIN SOLUTION
    return current + max(-max_step, min(max_step, target - current))
    ### END SOLUTION


# %% [check 2.5]
check_cases(
    "2.5",
    ramp_setpoint,
    [
        ((0.0, 1.0, 0.1), 0.1),
        ((0.95, 1.0, 0.1), 1.0),
        ((2.0, -1.0, 0.5), 1.5),
        ((1.0, 1.0, 0.2), 1.0),
    ],
)
fig, ax = plt.subplots(figsize=(7, 3.0))
for use_ramp, c, name in (
    (False, SERIES[1], "setpoint step"),
    (True, SERIES[0], "ramped, 0.02 per sample"),
):
    state = {"sp": 0.0}

    def target(t):
        return 1.0 if t >= 1.0 else 0.0

    def sp_fn(t, use_ramp=use_ramp, state=state):
        state["sp"] = (
            run("2.5", ramp_setpoint, state["sp"], target(t), 0.02) if use_ramp else target(t)
        )
        return state["sp"]

    r = simulate_pid(
        second_order(1.0, 2.0, 0.2), ep.PIDController(3.0, 2.0, 1.5, -50, 50), sp_fn, 3.0, 0.01
    )
    ax.plot(r.t, r.u, color=c, label=name)
ax.set(
    xlabel="time (s)",
    ylabel="controller output",
    title="Derivative kick at t = 1 s, and the ramp that removes it",
)
_ = ax.legend()

# %% [markdown]
# ## 5. Measuring a step response
#
# The capstone is scored on four numbers, defined once and used everywhere in this
# course (`bootcamp.metrics`):
#
# * **rise time**: from 10 % to 90 % of the step;
# * **overshoot**: $100\,(y_{peak} - r)/(r - y_0)$ %, 0 if $y$ never passes $r$;
# * **settling time**: first time after which $|y - r|$ stays inside 2 % of the step;
# * **steady-state error**: $r$ minus the mean of the last 10 % of samples.
#
# Write them yourself (linear interpolation between samples is optional; the check allows
# one sample of slack).


# %% [exercise 2.6]
def my_step_metrics(t, y, r):
    """Return (rise_time, overshoot_pct, settling_time) of a step from y[0] to r."""
    t, y = np.asarray(t), np.asarray(y)
    ### BEGIN SOLUTION
    y0 = y[0]
    s = (y - y0) / (r - y0)
    t10 = t[np.argmax(s >= 0.1)]
    t90 = t[np.argmax(s >= 0.9)]
    overshoot = max(0.0, s.max() - 1.0) * 100
    outside = np.nonzero(np.abs(s - 1.0) > 0.02)[0]
    settling = 0.0 if len(outside) == 0 else t[min(outside[-1] + 1, len(t) - 1)] - t[0]
    return t90 - t10, overshoot, settling
    ### END SOLUTION


# %% [check 2.6]
dt = 0.01
for g in ((3.0, 2.0, 1.5), (6.0, 2.0, 0.6), (1.0, 1.0, 0.3)):
    r = simulate_pid(second_order(1.0, 2.0, 0.2), ep.PIDController(*g, -10, 10), 1.0, 10.0, dt)
    ref = step_info(r.t, r.y, 1.0)
    rise, ovs, settle = run("2.6", my_step_metrics, r.t, r.y, 1.0)
    check_close("2.6", rise, ref.rise_time, atol=1.5 * dt, what=f"rise time, gains {g}")
    check_close("2.6", ovs, ref.overshoot_pct, atol=0.5, what=f"overshoot, gains {g}")
    check_close("2.6", settle, ref.settling_time, atol=1.5 * dt, what=f"settling time, gains {g}")

# %% [markdown]
# ## 6. Identify, then tune: the reaction curve
#
# Lab 2 uses a **3-stage RC ladder** (three 10 kΩ / 100 µF sections). Its transfer function
# is $1/(1 + 6sRC + 5(sRC)^2 + (sRC)^3)$ -- third order, but its step response looks like a
# first-order lag with a dead time. That **FOPDT** model, $K e^{-Ls}/(\tau s + 1)$, is the
# input of the classic tuning rules. The **two-point method**: find the times $t_{28}$ and
# $t_{63}$ at which the response covers 28.3 % and 63.2 % of its change; then
#
# $$\tau = 1.5\,(t_{63} - t_{28}), \qquad L = t_{63} - \tau, \qquad K = \Delta y / \Delta u .$$


# %% [exercise 2.7]
def fit_fopdt(t, y, du):
    """Two-point FOPDT fit of an open-loop step of size du applied at t[0]. Returns (K, tau, L)."""
    t, y = np.asarray(t) - t[0], np.asarray(y)
    ### BEGIN SOLUTION
    n = max(1, len(y) // 10)
    dy = y[-n:].mean() - y[0]
    s = (y - y[0]) / dy
    t28 = t[np.argmax(s >= 0.283)]
    t63 = t[np.argmax(s >= 0.632)]
    tau = 1.5 * (t63 - t28)
    return dy / du, tau, t63 - tau
    ### END SOLUTION


# %% [check 2.7]
ladder = rc_ladder(10e3, 100e-6, 3)
t_step, y_step, _ = simulate_open_loop(ladder, 2.0, 40.0, 0.02)  # a 0 -> 2 V step, 40 s
y_noisy = y_step + np.random.default_rng(7).normal(0, 0.003, len(y_step))  # ~ one ADC count
K, tau, L = run("2.7", fit_fopdt, t_step, y_noisy, 2.0)
ref = fit_fopdt_two_point(t_step, y_step, 2.0)
check_close("2.7", K, ref.K, rtol=0.02, what="static gain K")
check_close("2.7", tau, ref.tau, rtol=0.05, what="time constant tau (s)")
check_close("2.7", L, ref.L, rtol=0.10, atol=0.05, what="dead time L (s)")
fig, ax = plt.subplots(figsize=(7, 3.0))
ax.plot(t_step, y_noisy, color=SERIES[0], lw=1, label="3-stage ladder (simulated, noisy)")
fo = np.where(t_step < L, 0.0, 2.0 * K * (1 - np.exp(-(t_step - L) / tau)))
ax.plot(
    t_step, fo, color=SERIES[1], ls="--", label=f"FOPDT: K={K:.2f}, tau={tau:.2f} s, L={L:.2f} s"
)
ax.set(
    xlabel="time (s)",
    ylabel="volts",
    title="A third-order plant, seen as first order plus dead time",
)
_ = ax.legend()

# %% [markdown]
# With $K$, $\tau$, $L$ the open-loop Ziegler-Nichols rule (as written in easyPID's
# `docs/tuning_guide.md`) gives $K_p = 1.2\tau/(KL)$, $K_i = K_p/(2L)$, $K_d = 0.5K_pL$.
#
# ## 7. Relay autotuning with easyPID's `PIDTuner`
#
# Instead of a step, `PIDTuner` replaces the controller by a **relay** (bang-bang) of
# amplitude $d$ around an operating point. Any plant with enough lag then oscillates at
# the frequency where its phase is -180°, the *ultimate* frequency. With $a$ the measured
# half peak-to-peak amplitude and $h$ the noise band (relay hysteresis), the describing
# function of the relay gives the **ultimate gain**
#
# $$K_u = \frac{4d}{\pi\sqrt{a^2 - h^2}},$$
#
# and $P_u$ is the period of the oscillation. Tuning rules then map $(K_u, P_u)$ to gains
# (Ziegler-Nichols: $K_p = 0.6K_u$, $K_i = 1.2K_u/P_u$, $K_d = 0.075K_uP_u$).


# %% [exercise 2.8]
def relay_ultimate_gain(d, a, h=0.0):
    """Ultimate gain from a relay test: relay amplitude d, oscillation amplitude a, noise band h."""
    ### BEGIN SOLUTION
    return 4 * d / (math.pi * math.sqrt(a * a - h * h))
    ### END SOLUTION


# %% [check 2.8]
check_cases(
    "2.8",
    relay_ultimate_gain,
    [((1.0, 4 / (math.pi * 29)), 29.0), ((2.0, 0.1, 0.0), 25.4648), ((2.0, 0.1, 0.03), 26.6942)],
    rtol=1e-4,
)

# %% [markdown]
# For the RC ladder the exact answer is known: $K_u = 29$ at $\omega_u = \sqrt 6/RC$
# ($P_u = 2.57$ s). Here is the easyPID tuner (the Python port, line for line the C++ code)
# on a simulated ladder with ADC quantisation, run twice in a row:


# %%
def relay_runs(noise_band, relay_amp=2.0, runs=2):
    """Run easyPID's tuner `runs` times back to back on a simulated ladder (10-bit ADC)."""
    clock = ep.VirtualClock()
    pid = ep.PIDController(1.0, 0.0, 0.0, 0.0, 5.0, clock=clock)
    tuner = ep.PIDTuner(pid)
    pid.begin()
    ladder = rc_ladder(10e3, 100e-6, 3)
    y = 0.0
    for _ in range(4000):  # settle at the operating point, 2.5 V in, 2.5 V out
        y = ladder.step(2.5, 0.01)
    log, results = {"t": [], "y": [], "u": []}, []
    for _ in range(runs):
        tuner.start(2.5, relay_amp, noise_band, 2.5)  # setpoint, amplitude, noise band, bias
        while tuner.getState() == ep.TUNER_RELAY_STEP:
            u = tuner.update(round(y / 5 * 1023) * 5 / 1023)  # what analogRead sees
            for key, val in (("t", clock.millis() / 1000), ("y", y), ("u", u)):
                log[key].append(val)
            clock.advance(10)
            y = ladder.step(u, 0.01)
        results.append((tuner.getUltimateGain(), tuner.getUltimatePeriod()))
    return tuner, log, results


tuner, log, results = relay_runs(noise_band=0.0)
for k, (Ku, Pu) in enumerate(results, 1):
    print(f"run {k}: Ku = {Ku:.1f}, Pu = {Pu:.2f} s")
print(f"theory: Ku = 29.0, Pu = {2 * math.pi / math.sqrt(6):.2f} s")
fig, ax = plt.subplots(figsize=(9, 3.0))
ax.plot(log["t"], log["y"], color=SERIES[0], label="ladder output")
ax.plot(
    log["t"], np.array(log["u"]) / 20 + 2.375, color=SERIES[1], lw=1, label="relay output (scaled)"
)
ax.set(
    xlabel="time (s)",
    ylabel="volts",
    title="easyPID PIDTuner on the RC ladder: two runs back to back",
)
ax.legend()
save(fig, "s02_relay_autotune")

# %% [markdown]
# **The first run is biased** (higher $K_u$, shorter $P_u$): easyPID averages the cycles
# from the first relay switch on, while the oscillation of this slow plant is still
# growing. The second run, started while the plant is already oscillating, lands within a
# few percent of the theory -- the remaining error is the describing-function
# approximation. The lab-2 sketch (`s02_rc_tuning`) therefore runs the tuner twice and
# reports the second result. This is the kind of thing you only learn by testing a tool
# against a case where you know the answer.
#
# On real hardware the noise band cannot be zero, or noise makes the relay chatter. But
# the noise band is relay **hysteresis**: it shifts the oscillation to where the plant's
# phase is $-180° + \arcsin(h/a)$, a lower frequency, and this plant's gain changes fast
# near $\omega_u$:

# %%
print(" noise band h | h/a  | Ku (run 2) | Pu (run 2)")
for h in (0.0, 0.005, 0.01, 0.02):
    _, _, res = relay_runs(noise_band=h)
    Ku, Pu = res[-1]
    a = 4 * 2.0 / (math.pi * Ku)  # the amplitude the tuner measured
    print(f"   {1000 * h:4.0f} mV    | {h / a:.2f} | {Ku:8.1f}   | {Pu:6.2f} s")

# %% [markdown]
# So in the lab: use the **largest relay amplitude** the plant tolerates (that makes $a$
# large) and the **smallest noise band** that stops chatter (5 mV is one ADC count, just
# above the noise after averaging); expect $K_u$ to come out 10-20 % low. The rules below
# are a starting point either way.
#
# Now compare the tuning rules on the real ladder model:

# %%
ok, *_ = tuner.getTunings(ep.TUNING_ZIEGLER_NICHOLS)
assert ok
fig, ax = plt.subplots(figsize=(8, 3.4))
rules = (
    (ep.TUNING_ZIEGLER_NICHOLS, "Ziegler-Nichols"),
    (ep.TUNING_TYREUS_LUYBEN, "Tyreus-Luyben"),
    (ep.TUNING_PESSEN, "Pessen"),
    (ep.TUNING_NO_OVERSHOOT, "no overshoot"),
)
for (rule, name), c in zip(rules, SERIES):
    _, kp, ki, kd = tuner.getTunings(rule)
    r = simulate_pid(
        rc_ladder(10e3, 100e-6, 3), ep.PIDController(kp, ki, kd, 0.0, 5.0), 2.5, 25.0, 0.01
    )
    info = r.step_info()
    ax.plot(r.t, r.y, color=c, label=f"{name}: overshoot {info.overshoot_pct:.0f} %")
ax.axhline(2.5, **REF, label="setpoint")
ax.set(
    xlabel="time (s)",
    ylabel="volts",
    title="Four tuning rules from one relay test (3-stage RC ladder)",
)
ax.legend(fontsize=8)
save(fig, "s02_tuning_rules")
print(
    "reaction-curve rule on the FOPDT fit:",
    np.round(ziegler_nichols_open_loop(fit_fopdt_two_point(t_step, y_step, 2.0)), 3),
)

# %% [markdown]
# ### Reflection (not auto-checked)
#
# Name one plant you would **not** autotune with a relay, and say why.
#
# <!-- BEGIN SOLUTION -->
# Anything that is unstable or poorly damped open loop, or that cannot tolerate a sustained
# oscillation: the seesaw rig of session 3 (the relay would bang it against the stops), a
# drone in flight, a robot arm near people, a process where overshoot damages the product.
# The relay test also needs a plant with enough phase lag to oscillate at a well-sampled
# frequency; a nearly pure first-order lag oscillates at the sampling limit and the result
# is meaningless.
# <!-- END SOLUTION -->
#
# **Summary.** Sample 10-20x faster than the loop bandwidth. Keep `ANTIWINDUP_CLAMP` on
# unless you have a reason. Pick the EMA `alpha` from a cut-off frequency, not by habit.
# Ramp setpoints for loops with D. Identify before you tune, and test your tools on a case
# with a known answer.
