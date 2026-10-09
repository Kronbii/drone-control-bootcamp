# %% [markdown]
# # Session 6 · When things break: fault detection and fault-tolerant control
#
# **Drone-control bootcamp at Space²** · notebook 6 of 7
#
# A quadrotor has exactly as many motors as it has things to control (thrust and three
# torques). Chip a propeller and the controller is suddenly wrong about what its commands
# do; lose one completely and *no* allocation can hold it level with yaw fixed. This
# notebook goes from the arithmetic of that statement to a full fault-tolerant pipeline,
# using `rotorfault` -- Rami Kronbi's simulation study of rotor-fault detection, isolation
# and recovery for a 5-inch quad -- as the research-grade simulator.
#
# By the end you can
#
# 1. say, from the thrust-to-weight ratio alone, when hover with yaw held becomes impossible;
# 2. build fault signatures and isolate the faulty rotor from a residual;
# 3. implement a CUSUM test and reason about the detection-delay / false-alarm trade-off;
# 4. re-allocate thrust with an estimated effectiveness;
# 5. run the rotorfault pipeline, read its results, and explain the yaw-relaxed idea for a
#    complete rotor loss -- and what the study does *not* show.
#
# Requires `rotorfault` (see `docs/setup.md`).

# %%
import math

import matplotlib.pyplot as plt
import numpy as np

from bootcamp import fault, quad
from bootcamp.checks import check_cases, check_close, check_true, run
from bootcamp.plotting import REF, SERIES, save

fault.require_rotorfault()
from rotorfault.ftc import solve_relaxed_hover
from rotorfault.params import nominal_params
from rotorfault.rotations import tilt_angle

P = quad.QuadParams()
B = P.mixer()

# %% [markdown]
# ## 1. Loss of effectiveness, and the hover limit
#
# The fault model of the study is a multiplicative **loss of effectiveness** (LOE): rotor
# $i$ delivers $\eta_i f_i$ instead of $f_i$, with $\eta_i = 1$ healthy and $\eta_i = 0$ a
# complete loss (broken prop, dead ESC channel). The vehicle now produces
# $[T, \tau] = B\,\mathrm{diag}(\eta)\,f$.
#
# In hover with **zero yaw torque**, $B$ is invertible and each rotor must deliver exactly
# $mg/4$. A weakened rotor can only do that if $\eta_j f_{max} \ge mg/4$:
#
# $$\eta_j \ \ge\ \eta_{min} = \frac{mg}{4 f_{max}} = \frac{1}{T/W}.$$
#
# Below $\eta_{min}$ no controller can hold the vehicle level *and* stop it spinning.


# %% [exercise 6.1]
def eta_min(thrust_to_weight):
    """Smallest effectiveness of one rotor that still allows hover with yaw held."""
    ### BEGIN SOLUTION
    return 1.0 / thrust_to_weight
    ### END SOLUTION


# %% [check 6.1]
check_cases(
    "6.1",
    eta_min,
    [((4.0,), 0.25), ((2.0,), 0.5), ((P.thrust_to_weight,), P.hover_thrust / P.f_max)],
)
print(
    f"5-inch quad of the study: T/W = {P.thrust_to_weight:.2f} -> a rotor below eta = {eta_min(P.thrust_to_weight):.2f} cannot carry its share"
)
print(
    "(the study switches to yaw-relaxed control below eta = 0.35, leaving margin for attitude control)"
)

# %% [markdown]
# What a passive controller does about it -- the session-5 cascade, which knows nothing
# about faults, after a loss on rotor 1 at t = 1 s:

# %%
fig, ax = plt.subplots(figsize=(8, 3.2))
for loe, c in zip((0.3, 0.6, 1.0), SERIES):
    eta = 1.0 - loe
    lg = quad.simulate_quad(
        quad.CascadedController(),
        quad.hover_at((0, 0, 1)),
        4.0,
        eta_schedule=lambda t, e=eta: np.array([e if t >= 1.0 else 1.0, 1, 1, 1]),
    )
    err = np.linalg.norm(lg.p - lg.p_ref, axis=1)
    ax.plot(
        lg.t,
        100 * err,
        color=c,
        label=f"LOE {100 * loe:.0f} %"
        + (f" (lost at {lg.crash_time:.2f} s)" if lg.crashed else ""),
    )
ax.axvline(1.0, **REF, label="fault")
ax.set(
    xlabel="time (s)",
    ylabel="position error (cm)",
    title="Passive cascade (session 5) after a rotor-1 fault in hover",
)
ax.legend()
save(fig, "s06_passive_minimal")

# %% [markdown]
# Integral action absorbs a moderate loss, after a transient. A complete loss flips the
# vehicle in a fraction of a second. To do better we must **detect** the fault, **isolate**
# the rotor, **estimate** how bad it is, and **reconfigure**.
#
# ## 2. Residuals and fault signatures
#
# A residual compares what the vehicle does with what the healthy model predicts. Put the
# measured thrust and torques (from accelerometer, differentiated gyro and the known
# inertia) next to the model's prediction from the commanded rotor thrusts $f$:
#
# $$r = \underbrace{B\,\mathrm{diag}(\eta)\,f}_{\text{measured}} - \underbrace{B f}_{\text{predicted}} + \text{noise} = -\delta_j\,B_{:,j}\,f_j + \text{noise}$$
#
# for a loss $\delta_j = 1 - \eta_j$ on rotor $j$. So a fault on rotor $j$ pushes the
# residual along a known direction, its **signature** $d_j = -B_{:,j} f_j$. (rotorfault
# builds this residual from filtered IMU signals and a motor model, and normalises each
# row by its noise level; the geometry is the same.)


# %% [exercise 6.2]
def loe_signatures(B, f):
    """Matrix D whose column j is the residual direction of a loss on rotor j: d_j = -B[:, j] f_j."""
    ### BEGIN SOLUTION
    return -np.asarray(B) * np.asarray(f)[None, :]
    ### END SOLUTION


# %% [check 6.2]
f_hover = np.full(4, P.hover_thrust)
D = run("6.2", loe_signatures, B, f_hover)
check_close("6.2", D[:, 2], -B[:, 2] * P.hover_thrust, what="signature of rotor 3")
check_close("6.2", D, -(B * f_hover), what="all signatures")

# %% [markdown]
# **Isolation.** Given a residual, which rotor is it? Project it on each normalised
# signature, $t_j = d_j^T r / \lVert d_j \rVert$, and pick the largest. (With white noise
# of unit variance, $t_j$ is the matched-filter statistic; it is what feeds the CUSUM.)


# %% [exercise 6.3]
def isolate_rotor(r, D):
    """Index (0-3) of the signature best aligned with r."""
    ### BEGIN SOLUTION
    t = D.T @ r / np.linalg.norm(D, axis=0)
    return int(np.argmax(t))
    ### END SOLUTION


# %% [check 6.3]
rng = np.random.default_rng(6)
D_ref = -(B * f_hover)  # the signatures (exercise 6.2)
sigma = np.array([0.17, 4.5e-3, 4.5e-3, 2.4e-3])  # the study's residual noise floor per row
correct = 0
for _ in range(200):
    j, delta = rng.integers(4), rng.uniform(0.2, 0.9)
    r = delta * D_ref[:, j] + sigma * rng.standard_normal(4)
    rn, Dn = r / sigma, D_ref / sigma[:, None]  # normalise rows by their noise, as rotorfault does
    got = run("6.3", isolate_rotor, rn, Dn)
    check_close("6.3", got, fault.isolate(rn, Dn), what="isolated rotor", quiet=True)
    correct += got == j
check_true("6.3", correct >= 190, f"isolated the right rotor in {correct}/200 noisy single samples")

# %% [markdown]
# ## 3. Detection with CUSUM
#
# One noisy sample is not enough to *decide* that something broke. The **CUSUM** test of
# Page (1954) accumulates evidence for a mean shift from 0 to $\mu_1$ in noise of standard
# deviation $\sigma$, and forgets evidence against it:
#
# $$g_k = \max\!\left(0,\; g_{k-1} + \frac{\mu_1}{\sigma^2}\Big(r_k - \frac{\mu_1}{2}\Big)\right), \qquad \text{alarm when } g_k > h.$$
#
# Each sample adds the log-likelihood ratio of "shifted" vs "not shifted". The threshold
# $h$ sets the trade-off: larger $h$, fewer false alarms, longer detection delay.


# %% [exercise 6.4]
def my_cusum(r, mu1, sigma, h):
    """Return (index of the first alarm or -1, array of g_k)."""
    ### BEGIN SOLUTION
    g = np.zeros(len(r))
    acc, k_alarm = 0.0, -1
    for k, rk in enumerate(r):
        acc = max(0.0, acc + mu1 / sigma**2 * (rk - mu1 / 2))
        g[k] = acc
        if k_alarm < 0 and acc > h:
            k_alarm = k
    return k_alarm, g
    ### END SOLUTION


# %% [check 6.4]
for seed, shift_at in ((1, 500), (2, 1500), (3, None)):
    x = np.random.default_rng(seed).normal(0, 1, 3000)
    if shift_at is not None:
        x[shift_at:] += 0.8
    k_ref, g_ref = fault.cusum(x, 0.8, 1.0, 10.0)
    k, g = run("6.4", my_cusum, x, 0.8, 1.0, 10.0)
    check_close("6.4", g, g_ref, atol=1e-9, what="CUSUM statistic", quiet=True)
    check_close("6.4", k, k_ref, what=f"alarm index (shift at {shift_at})")

# %%
hs = np.linspace(2, 16, 8)
rng = np.random.default_rng(11)
delays, false_rate = [], []
for h in hs:
    d_list, fa = [], 0
    for _ in range(150):
        x = rng.normal(0, 1, 1200)
        k0, _ = fault.cusum(x[:1000], 0.8, 1.0, h)  # 1000 fault-free samples
        fa += k0 >= 0
        x[1000:] += 0.8
        k1, _ = fault.cusum(x[1000:], 0.8, 1.0, h)
        d_list.append(k1 if k1 >= 0 else np.nan)
    delays.append(np.nanmedian(d_list))
    false_rate.append(fa / 150)
fig, ax = plt.subplots(figsize=(6, 3.2))
ax.plot(delays, 100 * np.array(false_rate), "o-", color=SERIES[2], label="threshold h from 2 to 16")
for h, dl, fr in list(zip(hs, delays, false_rate))[::2]:
    ax.annotate(
        f"h={h:.0f}",
        (dl, 100 * fr),
        textcoords="offset points",
        xytext=(6, 4),
        fontsize=8,
        color="#b3b8b2",
    )
ax.set(
    xlabel="median detection delay (samples)",
    ylabel="runs with a false alarm (%)",
    title="CUSUM: buy fewer false alarms with delay",
)
ax.legend()
save(fig, "s06_cusum_tradeoff")

# %% [markdown]
# The rotorfault study runs **a bank** of CUSUM tests per rotor, designed for 15, 30 and
# 60 % LOE, because a single test designed for small faults accumulates evidence of a large
# fault far too slowly (its ablation, E7, shows the median delay doubling). Its thresholds
# are calibrated on fault-free flights rather than from formulas, because the real residual
# is coloured by turbulence and model error, not white.
#
# ## 4. Reconfiguration: allocate with the estimate
#
# Once rotor $j$ is isolated and its effectiveness estimated as $\hat\eta_j$, allocate
# through the *true* effectiveness matrix: solve $B\,\mathrm{diag}(\hat\eta)\,f = [T, \tau]$.


# %% [exercise 6.5]
def reallocate(B, eta_hat, T, tau):
    """Rotor commands f with B @ diag(eta_hat) @ f = [T, *tau]."""
    ### BEGIN SOLUTION
    return np.linalg.solve(np.asarray(B) * np.asarray(eta_hat)[None, :], np.concatenate([[T], tau]))
    ### END SOLUTION


# %% [check 6.5]
for eta_hat in (np.array([0.5, 1, 1, 1]), np.array([1, 0.7, 1, 1]), np.array([1, 1, 1, 0.4])):
    w = np.array([P.weight, 0.01, -0.02, 0.003])
    f_new = run("6.5", reallocate, B, eta_hat, w[0], w[1:])
    check_close(
        "6.5",
        (B * eta_hat[None, :]) @ f_new,
        w,
        atol=1e-9,
        what=f"wrench produced with eta = {eta_hat}",
    )
f_new = reallocate(B, np.array([0.5, 1, 1, 1]), P.weight, np.zeros(3))
print(
    "hover with rotor 1 at 50 %: commands", np.round(f_new, 2), "N; rotor 1 must double its command"
)

# %% [markdown]
# ## 5. The full pipeline: rotorfault
#
# The study's flight stack runs an estimator (complementary filter + Kalman filter on
# motion-capture data), the FDI (filtered regression, a Kalman filter on $\eta$, the CUSUM
# bank), and a **supervisor**: NOMINAL → PARTIAL (allocate with $\hat\eta$) on an alarm,
# → RELAXED when $\hat\eta_j < 0.35$ for 30 ms. Three baselines share the same vehicle,
# noise and turbulence for a given seed: **passive** (faults ignored), **oracle** (told
# the truth) and **estimated** (the proposed pipeline). `bootcamp.fault.run_case` wraps one
# simulation and returns the raw result plus a summary.
#
# A 60 % loss on rotor 1 at t = 2 s, in hover:

# %%
runs = {
    b: fault.run_case(0.6, rotor=1, baseline=b, duration=5.0, onset=2.0, seed=3)
    for b in ("passive", "estimated")
}
for b, (_, s) in runs.items():
    print(
        f"{b:9s}: lost={s.lost}, max deviation {s.max_deviation_cm:.1f} cm, detection {s.detection_delay_ms:.0f} ms, isolated rotor {s.isolated_rotor}"
    )
r_est = runs["estimated"][0]
fig, axs = plt.subplots(1, 3, figsize=(12, 3.2))
for (b, (r, _)), c in zip(runs.items(), SERIES):
    axs[0].plot(r.t, 100 * np.linalg.norm(r.position - r.p_ref, axis=1), color=c, label=b)
axs[1].plot(r_est.t, r_est.eta_hat[:, 0], color=SERIES[1], label="estimated eta, rotor 1")
axs[1].plot(r_est.t, r_est.eta_true[:, 0], **REF, label="true eta, rotor 1")
axs[2].plot(r_est.t, r_est.cusum[:, 0], color=SERIES[2], label="rotor 1")
axs[2].plot(r_est.t, r_est.cusum[:, 1:].max(axis=1), color=SERIES[3], label="rotors 2-4 (max)")
axs[2].axhline(1.0, **REF, label="alarm")
axs[2].set_xlim(1.5, 3.0)
axs[0].set(xlabel="time (s)", ylabel="cm", title="Position error, 60 % LOE at t = 2 s")
axs[1].set(xlabel="time (s)", ylabel="eta", title="Effectiveness estimate (Kalman filter)")
axs[2].set(xlabel="time (s)", ylabel="normalised CUSUM", title="Detection and isolation")
for a in axs:
    a.legend(fontsize=8)
save(fig, "s06_partial_loss")


# %% [exercise 6.6]
def detection_delay_ms(result, onset):
    """Alarm time minus fault onset in ms, from a rotorfault SimResult (nan if no alarm)."""
    ### BEGIN SOLUTION
    return (result.alarm_time - onset) * 1000.0 if math.isfinite(result.alarm_time) else math.nan
    ### END SOLUTION


# %% [check 6.6]
check_close(
    "6.6",
    run("6.6", detection_delay_ms, r_est, 2.0),
    runs["estimated"][1].detection_delay_ms,
    rtol=1e-9,
    what="detection delay (ms)",
    hint="SimResult.alarm_time is in seconds",
)
r0, s0 = fault.run_case(0.0, baseline="estimated", duration=3.0, seed=4)
check_true(
    "6.6",
    math.isnan(run("6.6", detection_delay_ms, r0, 2.0)),
    "no alarm (nan) in a fault-free flight",
)

# %% [markdown]
# ## 6. Complete loss of a rotor: give up yaw
#
# With $\eta_j = 0$ the argument of section 1 says hover with yaw held is impossible. Mueller
# and D'Andrea (2014) showed that it is possible to keep **position** if the vehicle is
# allowed to **spin**: the three remaining rotors produce a thrust along a body-fixed
# *primary axis* that is tilted from body z, the body spins about it fast enough that
# gyroscopic and drag torques balance, and on average the thrust points up. Control then
# acts only on the direction of that axis (a "reduced attitude") and on the thrust.
#
# rotorfault solves this **relaxed-hover equilibrium** numerically:

# %%
eq = solve_relaxed_hover(nominal_params(), failed=0, tilt_deg=15.0)
print(f"rotor 1 failed: rotor thrusts {np.round(eq.f, 2)} N (max {P.f_max:.1f} N)")
print(
    f"spin |omega| = {eq.spin:.1f} rad/s = {math.degrees(eq.spin):.0f} deg/s (FPV gyro range 2000 deg/s)"
)
print(
    f"primary axis (body frame) {np.round(eq.n, 3)}, tilted {eq.axis_tilt_deg:.1f} deg from body z"
)
print(
    f"total thrust {eq.total_thrust:.2f} N vs weight {P.weight:.2f} N: n_z * T must equal the weight"
)

# %%
loss = {
    b: fault.run_case(1.0, rotor=1, baseline=b, duration=6.0, onset=2.0, seed=3)
    for b in ("passive", "estimated")
}
for b, (_, s) in loss.items():
    extra = (
        f"lost ({s.lost_reason})"
        if s.lost
        else f"recovered, peak deviation {s.max_deviation_cm:.0f} cm, spin {s.final_spin_rad_s:.1f} rad/s, switch after {s.switch_to_relaxed_ms:.0f} ms"
    )
    print(f"{b:9s}: {extra}")
fig, axs = plt.subplots(1, 3, figsize=(12, 3.3))
for (b, (r, s)), c in zip(loss.items(), SERIES):
    lab = b + (" (lost)" if s.lost else "")
    axs[0].plot(r.t, 100 * np.linalg.norm(r.position - r.p_ref, axis=1), color=c, label=lab)
    axs[1].plot(r.t, [math.degrees(tilt_angle(q)) for q in r.quat], color=c, label=lab)
    axs[2].plot(r.t, r.omega[:, 2], color=c, label=lab)
for a in axs:
    a.axvline(2.0, **REF, label="rotor 1 lost")
    a.legend(fontsize=8)
axs[0].set(xlabel="time (s)", ylabel="cm", title="Position error")
axs[0].set_ylim(0, 120)
axs[1].set(xlabel="time (s)", ylabel="deg", title="Tilt of body z")
axs[2].set(xlabel="time (s)", ylabel="rad/s", title="Yaw rate: the price of survival")
save(fig, "s06_complete_loss")

# %% [markdown]
# The passive controller flips within about 0.3 s. The pipeline detects the loss in tens of
# milliseconds, switches to yaw-relaxed control about 60 ms after the failure, and holds
# position while spinning at over 20 rad/s.
#
# ## 7. Read the limits before you believe the plots
#
# The study's own limitations section (README §6 of `rotor-fault-recovery`) is part of
# this session. In short: simulation only, with **assumed** (not identified) parameters; a
# white-noise vibration model much kinder than a real 5-inch frame; motion capture with
# 10 ms latency; one fault at a time, as a clean step; and recovery that **broke** in its
# robustness sweep at 24 ms of actuator delay (2 ms assumed), at a motor time constant of
# 80 ms (25 ms assumed) and at 3x the default turbulence.
#
# ### Reflection (not auto-checked)
#
# You want to test this pipeline on a real 5-inch quad next year. Name the first three
# measurements you would make *before* any flight test, and say which result of the study
# each one could invalidate.
#
# <!-- BEGIN SOLUTION -->
# 1. **Actuator delay and motor time constant** (log commanded vs measured RPM with
#    bidirectional DShot on a test stand): the robustness sweep shows complete-loss recovery
#    failing at 24 ms delay or 80 ms motor lag, so these decide whether yaw-relaxed control
#    can work at all on that airframe.
# 2. **The real residual floor**: run the diagnoser offline on blackbox logs of fault-free
#    flights (gyro, motor outputs, RPM). Narrow-band prop vibration is much larger than the
#    simulated white noise; it sets the CUSUM thresholds and therefore the detection delay
#    and the false-alarm rate (E2, E7).
# 3. **Inertia, thrust and rotational-damping coefficients** (swing tests, thrust stand,
#    spin-down tests): the rotational damping sets the spin rate after a complete loss and is
#    the least certain number in the model; the spin must stay inside the gyro range.
# Only then: a protected rig, a deliberately weakened propeller (partial LOE), and only much
# later a sacrificial rotor cut -- the order the study itself proposes.
# <!-- END SOLUTION -->
#
# **Summary.** A quad has no spare actuator: below $\eta = 1/(T/W)$ hover with yaw held is
# impossible. Residuals plus known signatures give isolation; CUSUM turns noisy evidence
# into a decision; re-allocation uses the estimate; and a complete loss is survivable only
# by letting the vehicle spin. All of it is simulation until measured.
