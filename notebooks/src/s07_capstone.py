# %% [markdown]
# # Session 7 · Capstone: tune it, test it, report it
#
# **Drone-control bootcamp at Space²** · notebook 7 of 7
#
# Teams choose a track (details in `sessions/07-capstone/README.md`):
#
# * **Track R -- rig.** Tune a controller for the seesaw rig and pass the acceptance test
#   that `arduino/s07_capstone` runs on the `test` command: steps, a repeatable
#   disturbance pulse and an injected motor fault.
# * **Track S -- simulation.** Fly the quadrotor waypoint course of `bootcamp.capstone`
#   with a 50 % loss of effectiveness on rotor 1 half-way through, and make it fault
#   tolerant.
#
# Either way the deliverable is the same: numbers against a written specification, the
# plots that show them, and a two-page report that says honestly what worked. This
# notebook is the analysis toolkit. Work through sections 1-3 with the example log, then
# point it at **your** log.

# %%
import math

import matplotlib.pyplot as plt
import numpy as np

from bootcamp import capstone as cp
from bootcamp import easypid as ep
from bootcamp import quad
from bootcamp.checks import check_close, check_true, run
from bootcamp.logs import read_log
from bootcamp.plotting import REF, SERIES, repo_root, save

ROOT = repo_root()
TEAM_LOG = ROOT / "logs" / "team_capstone.csv"  # put your logger output here
EXAMPLE_LOG = ROOT / "data" / "example_capstone_rig.csv"
log_path = TEAM_LOG if TEAM_LOG.exists() else EXAMPLE_LOG
raw = read_log(log_path)
print(f"loaded {log_path.relative_to(ROOT)}: {len(raw['t_ms'])} rows, columns {list(raw)}")
if log_path == EXAMPLE_LOG:
    print(
        "NOTE: this is the SYNTHETIC example log (generated from the rig model), not a measurement."
    )

# %% [markdown]
# ## 1. From a raw log to the test window
#
# The sketch prints a row every 20 ms. `test_t` is the time since the `test` command in
# seconds, and **-1** outside the test (before it, after it, or if the rig disarmed). The
# analysis uses only the rows of the test, with `test_t` as the time axis.


# %% [exercise 7.1]
def extract_test(log):
    """Return (t, sp, angle, u) for the rows with test_t >= 0, using the pot as the angle."""
    ### BEGIN SOLUTION
    m = log["test_t"] >= 0
    return log["test_t"][m], log["sp"][m], log["pot"][m], log["u"][m]
    ### END SOLUTION


# %% [check 7.1]
t_s, sp_s, angle_s, u_s = run("7.1", extract_test, raw)
m = raw["test_t"] >= 0
check_close("7.1", t_s, raw["test_t"][m], what="test time")
check_close("7.1", angle_s, raw["pot"][m], what="angle (pot)")
check_true(
    "7.1",
    t_s[0] < 0.1 and t_s[-1] > cp.RIG_TEST_END - 0.5,
    f"the test window covers 0 to {t_s[-1]:.1f} s",
)

# %%
# The rest of the notebook uses the test window extracted here (same as exercise 7.1).
m = raw["test_t"] >= 0
t, sp, angle, u = raw["test_t"][m], raw["sp"][m], raw["pot"][m], raw["u"][m]

# %% [markdown]
# ## 2. The numbers of the specification
#
# The spec sheet (`bootcamp.capstone.RIG_SPEC`) uses the course-wide definitions of
# session 2 with one change: settling is measured into a **±1° band** (an absolute band
# is fairer than 2 % of 15° = 0.3°, about one pot count). Write the step evaluation, then
# compare with the reference implementation.


# %% [exercise 7.2]
def step_overshoot_and_settling(t, y, target, y0, band_deg):
    """Overshoot (%) of a step y0 -> target, and the time after t[0] from which |y - target|
    stays within band_deg (nan if it never settles)."""
    ### BEGIN SOLUTION
    s = (y - y0) / (target - y0)
    overshoot = max(0.0, float(s.max()) - 1.0) * 100.0
    out = np.nonzero(np.abs(y - target) > band_deg)[0]
    if len(out) == 0:
        settling = 0.0
    elif out[-1] == len(y) - 1:
        settling = math.nan
    else:
        settling = float(t[out[-1] + 1] - t[0])
    return overshoot, settling
    ### END SOLUTION


# %% [check 7.2]
w = (t >= 3.0) & (t < 8.0)
ovs, settle = run("7.2", step_overshoot_and_settling, t[w], angle[w], 15.0, 0.0, 1.0)
ref = cp.evaluate_rig_test(t, sp, angle, u)
check_close(
    "7.2", ovs, ref["step_overshoot_pct"], atol=1e-6, what="overshoot of the +15 deg step (%)"
)
check_close("7.2", settle, ref["step_settling_s"], atol=1e-6, what="settling into +-1 deg (s)")
check_close(
    "7.2",
    step_overshoot_and_settling(np.arange(5.0), np.array([0, 5, 12, 10.5, 10]), 10, 0, 1)[1],
    3.0,
    what="settling on a toy signal",
)

# %% [markdown]
# The **disturbance** is a 40 µs, 100 ms pulse on the differential command at
# t = 20 s; the **fault** scales motor 2's throttle above idle by 0.75 from t = 25 s, and a
# +10° step follows at t = 29 s. The peak deviation after the pulse is the largest $|\theta|$
# between 20 s and 25 s.


# %% [exercise 7.3]
def disturbance_peak(t, angle, t0, t1, target=0.0):
    """Largest |angle - target| for t0 <= t < t1."""
    ### BEGIN SOLUTION
    m = (t >= t0) & (t < t1)
    return float(np.abs(angle[m] - target).max())
    ### END SOLUTION


# %% [check 7.3]
check_close(
    "7.3",
    run("7.3", disturbance_peak, t, angle, cp.DIST_TIME, cp.FAULT_TIME),
    ref["dist_peak_deg"],
    what="disturbance peak (deg)",
)
check_close(
    "7.3",
    disturbance_peak(np.arange(6.0), np.array([0, 1, -3, 2, 9, 9]), 1, 4),
    3.0,
    what="toy signal",
)

# %%
results = cp.evaluate_rig_test(t, sp, angle, u)
table = cp.spec_check(results)
print(f"{'metric':22s} {'value':>9s} {'limit':>7s}  result")
for k, (v, lim, ok) in table.items():
    print(f"{k:22s} {v:9.2f} {lim:7.2f}  {'PASS' if ok else 'FAIL'}")
fig, axs = plt.subplots(2, 1, figsize=(10, 5.2), sharex=True)
axs[0].plot(t, angle, color=SERIES[0], label="beam angle (pot)")
axs[0].plot(t, sp, **REF, label="setpoint")
axs[0].axvspan(
    cp.DIST_TIME, cp.DIST_TIME + 0.1, color=SERIES[3], alpha=0.5, label="disturbance pulse"
)
axs[0].axvspan(cp.FAULT_TIME, cp.RIG_TEST_END, color=SERIES[1], alpha=0.10, label="motor-2 fault")
axs[0].set(ylabel="deg", title=f"Acceptance test, {log_path.name}")
axs[1].plot(t, u, color=SERIES[2], lw=1, label="differential command u")
axs[1].set(xlabel="test time (s)", ylabel="us")
axs[0].legend(ncol=4, fontsize=8)
axs[1].legend(fontsize=8)
save(fig, "s07_acceptance_test")

# %% [markdown]
# ## 3. Pre-flight: tune in simulation first (Track R)
#
# `cp.simulate_rig_test` runs the identical test on the rig model. Return gains for the
# sketch's cascade (outer angle P/PI, inner rate PID, pot as the angle source) that pass
# **every** line of the spec on the model, for two different noise seeds. Then put them
# into the sketch (`akp 5` ... over serial) and run the real test -- the model is only a
# model, so expect to retune.


# %% [exercise 7.4]
def capstone_gains():
    """Return (akp, aki, rkp, rki, rkd) for s07_capstone."""
    ### BEGIN SOLUTION
    return 5.0, 0.5, 1.0, 1.0, 0.02
    ### END SOLUTION


# %% [check 7.4]
def cascade(akp, aki, rkp, rki, rkd):
    outer = ep.PIDController(akp, aki, 0.0, -150, 150)
    inner = ep.PIDController(rkp, rki, rkd, -200, 200)
    inner.setDerivativeFilter(ep.FILTER_EMA, 0.7)
    outer.begin()
    inner.begin()
    return lambda t_, sp_, m_, dt: inner.update(
        outer.update(sp_, m_["pot"], dt * 1000), m_["gyro"], dt * 1000
    )


g = run("7.4", capstone_gains)
for seed in (1, 2):
    sim = cp.simulate_rig_test(cascade(*g), seed=seed)
    tab = cp.spec_check(cp.evaluate_rig_test(sim.t, sim.setpoint, sim.theta, sim.u))
    failed = [f"{k} = {v:.2f} (limit {lim})" for k, (v, lim, ok) in tab.items() if not ok]
    check_true(
        "7.4",
        not failed,
        f"seed {seed}: all {len(tab)} spec lines pass"
        if not failed
        else f"seed {seed} fails: {failed}",
    )

# %% [markdown]
# ## 4. Track S: the quadrotor course with a rotor fault
#
# Five waypoints, 4 s each; at t = 9 s rotor 1 loses half its effectiveness. The
# passive cascade of session 5 survives but tracks badly. A fault-tolerant controller
# needs (a) an estimate $\hat\eta$ and (b) an allocator that uses it. Part (b) is the
# exercise; for (a) the notebook uses an **oracle with a 50 ms delay** (it is told the
# truth 50 ms after the fault, about the detection delay of the rotorfault pipeline). The
# Track S project is to replace that oracle with your own detector from session 6.


# %%
class FaultAwareCascade(quad.CascadedController):
    """The session-5 cascade with an allocator that uses eta_hat (set from outside)."""

    def __init__(self, allocate_fn, **kw):
        super().__init__(**kw)
        self.eta_hat = np.ones(4)
        self.allocator = lambda B_, T_, tau_, lo, hi: allocate_fn(
            B_, self.eta_hat, T_, tau_, lo, hi
        )


course_ref = quad.waypoints(cp.COURSE, cp.COURSE_DWELL)
T_course = len(cp.COURSE) * cp.COURSE_DWELL


def fly_course(allocate_fn, oracle_delay=0.05):
    ctrl = FaultAwareCascade(allocate_fn)

    def eta_schedule(t_):
        if t_ >= cp.COURSE_FAULT_TIME + oracle_delay:
            ctrl.eta_hat = cp.course_eta(t_)  # oracle: replace with your detector + estimator
        return cp.course_eta(t_)

    return quad.simulate_quad(ctrl, course_ref, T_course, eta_schedule=eta_schedule)


# %% [exercise 7.5]
def fault_aware_allocate(B, eta_hat, T, tau, f_min, f_max):
    """Like quad.allocate (saturation priorities), but for a vehicle that produces
    B @ diag(eta_hat) @ f. Returns rotor commands f."""
    ### BEGIN SOLUTION
    return quad.allocate(np.asarray(B) * np.asarray(eta_hat)[None, :], T, tau, f_min, f_max)
    ### END SOLUTION


# %% [check 7.5]
Pq = quad.QuadParams()
Bq = Pq.mixer()
eta = np.array([0.5, 1, 1, 1])
wq = np.array([Pq.weight, 0.01, -0.01, 0.002])
fq = run("7.5", fault_aware_allocate, Bq, eta, wq[0], wq[1:], Pq.f_min, Pq.f_max)
check_close(
    "7.5", (Bq * eta[None, :]) @ fq, wq, atol=1e-9, what="wrench delivered by the damaged vehicle"
)
check_true(
    "7.5",
    fq.min() >= Pq.f_min - 1e-12 and fq.max() <= Pq.f_max + 1e-12,
    "commands within the motor limits",
)
res_mine = cp.evaluate_quad_course(fly_course(fault_aware_allocate))
print(
    "your allocator:", {k: (v if isinstance(v, bool) else round(v, 3)) for k, v in res_mine.items()}
)
check_true(
    "7.5",
    not res_mine["crashed"]
    and res_mine["max_error_after_fault_m"] < cp.QuadSpec().max_error_after_fault_m,
    "the fault-aware cascade meets the course spec",
)


# %%
def reference_allocate(B, eta_hat, T, tau, f_min, f_max):
    return quad.allocate(B * eta_hat[None, :], T, tau, f_min, f_max)


log_ftc = fly_course(reference_allocate)
log_passive = quad.simulate_quad(
    quad.CascadedController(), course_ref, T_course, eta_schedule=cp.course_eta
)
for name, lg in (("passive", log_passive), ("fault-aware", log_ftc)):
    print(
        f"{name:12s}",
        {
            k: (v if isinstance(v, bool) else round(v, 3))
            for k, v in cp.evaluate_quad_course(lg).items()
        },
    )
fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
for lg, c, name in (
    (log_passive, SERIES[1], "passive"),
    (log_ftc, SERIES[0], "fault-aware (oracle 50 ms)"),
):
    axs[0].plot(lg.p[:, 0], lg.p[:, 1], color=c, label=name)
    axs[1].plot(lg.t, 100 * np.linalg.norm(lg.p - lg.p_ref, axis=1), color=c, label=name)
wp = np.array(cp.COURSE)
axs[0].plot(wp[:, 0], wp[:, 1], "s", color="#b3b8b2", ms=6, label="waypoints")
axs[0].set(xlabel="x (m)", ylabel="y (m)", title="Track S course, 50 % LOE on rotor 1 at t = 9 s")
axs[0].set_aspect("equal", adjustable="datalim")
axs[1].axvline(cp.COURSE_FAULT_TIME, **REF, label="fault")
axs[1].set(xlabel="time (s)", ylabel="distance to waypoint (cm)", title="Tracking error")
axs[0].legend(fontsize=8)
axs[1].legend(fontsize=8)
save(fig, "s07_course")

# %% [markdown]
# ## 5. Your report
#
# Two pages, using `sessions/07-capstone/report-template.md`. It must contain:
#
# 1. the spec table above for **your** log, with every FAIL explained;
# 2. the acceptance-test plot, and one plot you chose because it shows something the
#    numbers do not;
# 3. your gains, how you found them, and how far the hardware result was from the
#    simulation (section 3) -- and your best explanation of the difference;
# 4. one thing you would change in the rig or the controller with another week.
#
# ### Reflection (not auto-checked)
#
# The example log passes most lines of the spec. Why is passing the acceptance test once
# not the same as "the controller works"?
#
# <!-- BEGIN SOLUTION -->
# One run is one sample: noise, battery or supply voltage, prop condition and temperature
# change between runs, so a margin that is small on one run can fail on the next (the
# check above uses two seeds for this reason; on the rig, run the test at least three
# times and report the worst). The test also covers only what it tests: one step size, one
# disturbance, one fault level. A controller tuned to pass it can still be fragile to a
# larger step, a heavier beam or a different motor, so the report should state the
# conditions and the margins, not just PASS.
# <!-- END SOLUTION -->
