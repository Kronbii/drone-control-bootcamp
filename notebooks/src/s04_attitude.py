# %% [markdown]
# # Session 4 · Rotations, attitude estimation and cascaded loops
#
# **Drone-control bootcamp at Space²** · notebook 4 of 7
#
# A drone does not have a potentiometer on its roll axis. It has an **IMU**: a gyroscope
# that measures angular *rate* and an accelerometer that measures *specific force*. Neither
# measures attitude. This notebook builds the attitude estimate a flight controller runs,
# tests it against the rig's pot (which you *do* have on the bench), and then closes the
# loop the way every flight controller does: a fast **rate** loop inside a slower
# **angle** loop.
#
# By the end you can
#
# 1. build rotation matrices from Z-Y-X Euler angles, and say where they break (gimbal lock);
# 2. multiply quaternions and rotate vectors with them;
# 3. get a tilt angle from an accelerometer and explain when it lies;
# 4. write a complementary filter, choose its time constant, and compare it with a Kalman filter;
# 5. tune a cascaded rate/angle controller and show why it rejects disturbances better.

# %%
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from bootcamp import attitude as att
from bootcamp import easypid as ep
from bootcamp import rig
from bootcamp.checks import check_close, check_less, check_true, run
from bootcamp.metrics import rms, step_info
from bootcamp.plotting import REF, SERIES, save

# %% [markdown]
# ## 1. Rotation matrices and Euler angles
#
# A rotation matrix $R$ maps vectors from the body frame to the world frame,
# $v_W = R\,v_B$. Its columns are the body axes written in world coordinates; it is
# orthonormal ($R^TR = I$, $\det R = 1$), so $R^{-1} = R^T$. The elementary rotations are
#
# $$R_x(\phi) = \begin{bmatrix}1&0&0\\0&\cos\phi&-\sin\phi\\0&\sin\phi&\cos\phi\end{bmatrix},\;
# R_y(\theta) = \begin{bmatrix}\cos\theta&0&\sin\theta\\0&1&0\\-\sin\theta&0&\cos\theta\end{bmatrix},\;
# R_z(\psi) = \begin{bmatrix}\cos\psi&-\sin\psi&0\\\sin\psi&\cos\psi&0\\0&0&1\end{bmatrix}.$$
#
# Drones (and this course, and the rotorfault study) use **Z-Y-X Euler angles**: yaw
# $\psi$, then pitch $\theta$, then roll $\phi$, each about the *new* axis:
# $R = R_z(\psi)\,R_y(\theta)\,R_x(\phi)$. Frames: world z up, body x forward, y left, z up.


# %% [exercise 4.1]
def my_euler_zyx(roll, pitch, yaw):
    """Rotation matrix R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (angles in rad)."""
    ### BEGIN SOLUTION
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx
    ### END SOLUTION


# %% [check 4.1]
rng = np.random.default_rng(4)
for _ in range(20):
    roll, pitch, yaw = (
        rng.uniform(-math.pi, math.pi),
        rng.uniform(-1.5, 1.5),
        rng.uniform(-math.pi, math.pi),
    )
    R = run("4.1", my_euler_zyx, roll, pitch, yaw)
    check_close(
        "4.1",
        R,
        Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix(),
        atol=1e-12,
        what="R",
        quiet=True,
        hint="order matters: Rz @ Ry @ Rx",
    )
    check_close("4.1", R.T @ R, np.eye(3), atol=1e-12, what="R^T R", quiet=True)
check_true("4.1", True, "my_euler_zyx agrees with scipy on 20 random attitudes and is orthonormal")

# %% [markdown]
# **Gimbal lock.** At pitch ±90° the first and last rotation axes line up, and roll and
# yaw can no longer be told apart: very different (roll, yaw) pairs give the same matrix.
# A controller written in Euler angles misbehaves near there -- one reason flight stacks
# keep attitude as a quaternion or a matrix.

# %%
R1 = att.euler_zyx_to_R(0.3, math.pi / 2, 0.0)
R2 = att.euler_zyx_to_R(0.0, math.pi / 2, -0.3)
print("roll 0.3, yaw 0  vs  roll 0, yaw -0.3 at pitch 90 deg: same matrix?", np.allclose(R1, R2))

# %% [markdown]
# ## 2. Quaternions, lightly
#
# A unit quaternion $q = (\cos\frac{\alpha}{2},\ \sin\frac{\alpha}{2}\,\mathbf n)$ is a rotation
# by $\alpha$ about the unit axis $\mathbf n$: four numbers, no singularity, cheap to
# integrate. Composition is the quaternion product (scalar-first, $q = (w, \mathbf v)$):
#
# $$q_a \otimes q_b = \big(w_a w_b - \mathbf v_a\cdot\mathbf v_b,\;\; w_a\mathbf v_b + w_b\mathbf v_a + \mathbf v_a\times\mathbf v_b\big),$$
#
# and a vector rotates as $v' = q \otimes (0, v) \otimes q^*$. The attitude is propagated
# from the gyro with $q_{k+1} = q_k \otimes \exp(\tfrac12 \omega\,\Delta t)$.


# %% [exercise 4.2]
def my_quat_mul(a, b):
    """Hamilton product of scalar-first quaternions a and b (arrays of 4)."""
    ### BEGIN SOLUTION
    wa, va = a[0], np.asarray(a[1:])
    wb, vb = b[0], np.asarray(b[1:])
    return np.concatenate([[wa * wb - va @ vb], wa * vb + wb * va + np.cross(va, vb)])
    ### END SOLUTION


# %% [check 4.2]
for _ in range(20):
    a, b = rng.normal(size=4), rng.normal(size=4)
    check_close(
        "4.2",
        run("4.2", my_quat_mul, a, b),
        att.quat_mul(a, b),
        atol=1e-12,
        what="product",
        quiet=True,
    )
qa = att.quat_from_axis_angle([0, 0, 1], math.pi / 2)
qb = att.quat_from_axis_angle([1, 0, 0], math.pi / 2)
q = my_quat_mul(qa, qb)  # first rotate about x, then about z
check_close(
    "4.2",
    att.quat_to_R(q),
    att.rot_z(math.pi / 2) @ att.rot_x(math.pi / 2),
    atol=1e-12,
    what="composed rotation",
)

# %% [markdown]
# ## 3. What the IMU on the rig measures
#
# The rig's MPU-6050 sits on the pivot axis with its x axis along the beam. At rest the
# accelerometer measures gravity's reaction, so in the beam's plane
#
# $$a_x = g\sin\theta, \quad a_z = g\cos\theta \quad\Longrightarrow\quad \theta_{acc} = \operatorname{atan2}(a_x, a_z).$$
#
# That is exact *only at rest*. Propeller vibration adds noise of a sizeable fraction of
# a g, and any linear acceleration of the sensor is indistinguishable from tilt. The gyro
# measures $\dot\theta$ cleanly, but integrating it accumulates its bias forever.
# Simulated rig, motors on, the beam moved by its controller (pot angle = truth):


# %%
def demo_run(sensors=None, seconds=20.0, seed=5, setpoint=None):
    pid = ep.PIDController(2.0, 1.5, 0.8, -200, 200)
    pid.setDerivativeFilter(ep.FILTER_EMA, 0.6)
    pid.begin()
    return rig.simulate_rig(
        lambda t, sp, m, dt: pid.update(sp, m["pot"], dt * 1000),
        seconds,
        setpoint=setpoint or (lambda t: 12.0 * math.sin(2 * math.pi * t / 8)),
        sensors=sensors,
        seed=seed,
    )


log = demo_run()
dt = log.t[1] - log.t[0]
gyro_int = np.cumsum(log.gyro) * dt
fig, ax = plt.subplots(figsize=(9, 3.2))
ax.plot(log.t, log.accel_angle, color=SERIES[1], lw=0.6, label="accelerometer only")
ax.plot(log.t, gyro_int, color=SERIES[3], label="gyro integrated")
ax.plot(log.t, log.theta, color=SERIES[0], lw=2, label="true angle")
ax.set(
    xlabel="time (s)", ylabel="beam angle (deg)", title="Noisy but unbiased vs smooth but drifting"
)
ax.legend(ncol=3)
save(fig, "s04_gyro_vs_accel")
print(
    f"accelerometer RMS error {rms(log.accel_angle - log.theta):.1f} deg; gyro drift after 20 s {gyro_int[-1] - log.theta[-1]:.1f} deg"
)


# %% [exercise 4.3]
def accel_angle_deg(ax, az):
    """Beam angle (deg) from the accelerometer components along (ax) and normal to (az) the beam."""
    ### BEGIN SOLUTION
    return np.degrees(np.arctan2(ax, az))
    ### END SOLUTION


# %% [check 4.3]
check_close(
    "4.3",
    run("4.3", accel_angle_deg, log.accel_x, log.accel_z),
    log.accel_angle,
    atol=1e-9,
    what="accelerometer angle",
    hint="np.arctan2(ax, az), in degrees",
)
check_close("4.3", accel_angle_deg(-0.5, 0.5), -45.0, what="angle for (ax, az) = (-0.5, 0.5)")

# %% [markdown]
# ## 4. The complementary filter
#
# Trust the gyro at high frequency and the accelerometer at low frequency:
#
# $$\hat\theta_k = \alpha\,(\hat\theta_{k-1} + \omega_k\,\Delta t) + (1-\alpha)\,\theta_{acc,k}, \qquad \alpha = \frac{\tau}{\tau + \Delta t}.$$
#
# $\tau$ is the crossover time constant: slower than $\tau$, the accelerometer wins; faster,
# the gyro. With a constant gyro bias $b$ the estimate settles at an error of about $b\,\tau$.


# %% [exercise 4.4]
def complementary(gyro, acc_angle, dt, tau, angle0=0.0):
    """Run the complementary filter over arrays; return the estimated angles (array)."""
    ### BEGIN SOLUTION
    alpha = tau / (tau + dt)
    out = np.empty(len(gyro))
    a = angle0
    for k in range(len(gyro)):
        a = alpha * (a + gyro[k] * dt) + (1 - alpha) * acc_angle[k]
        out[k] = a
    return out
    ### END SOLUTION


# %% [check 4.4]
cf = att.ComplementaryFilter(att.alpha_from_time_constant(0.5, dt))
ref = np.array([cf.update(g, a, dt) for g, a in zip(log.gyro, log.accel_angle)])
mine = run("4.4", complementary, log.gyro, log.accel_angle, dt, 0.5)
check_close("4.4", mine, ref, atol=1e-9, what="filter output")
err = rms(mine[log.t > 2] - log.theta[log.t > 2])
check_less("4.4", err, 1.5, "RMS error against the true angle", " deg")
fig, ax = plt.subplots(figsize=(9, 3.2))
ax.plot(log.t, log.accel_angle, color=SERIES[1], lw=0.5, alpha=0.6, label="accelerometer")
ax.plot(log.t, mine, color=SERIES[2], label=f"complementary, tau = 0.5 s (RMS err {err:.2f} deg)")
ax.plot(log.t, log.theta, color=SERIES[0], lw=1.2, ls="--", label="truth (pot)")
ax.set(xlabel="time (s)", ylabel="deg", title="Complementary filter on the rig, motors running")
ax.legend()
save(fig, "s04_complementary")

# %% [markdown]
# **Choosing τ.** Too small and accelerometer noise comes through; too large and the
# gyro bias leaks in ($b\tau$) and the filter recovers slowly. A sweep on the same data:

# %%
taus = np.geomspace(0.02, 10, 25)
errs = []
for tau in taus:
    f = att.ComplementaryFilter(att.alpha_from_time_constant(tau, dt))
    est = np.array([f.update(g, a, dt) for g, a in zip(log.gyro, log.accel_angle)])
    errs.append(rms(est[log.t > 2] - log.theta[log.t > 2]))
fig, ax = plt.subplots(figsize=(6, 3.0))
ax.semilogx(taus, errs, "o-", color=SERIES[2], ms=3, label="RMS error")
ax.set(
    xlabel="time constant tau (s)",
    ylabel="RMS error (deg)",
    title="Noise on the left, gyro bias on the right",
)
ax.legend()
save(fig, "s04_tau_sweep")
print(f"best tau on this data: {taus[int(np.argmin(errs))]:.2f} s")

# %% [markdown]
# ## 5. A preview of the Kalman filter
#
# The complementary filter has a fixed gain. A **Kalman filter** computes the best gain
# from noise models and can estimate the gyro bias as a second state, $x = [\theta, b]$:
#
# * predict: $\theta^- = \theta + (\omega - b)\Delta t$, $b^- = b$, $\;P^- = FPF^T + Q$ with
#   $F = \begin{bmatrix}1 & -\Delta t\\ 0 & 1\end{bmatrix}$, $Q = \mathrm{diag}(q_\theta\Delta t,\ q_b\Delta t)$;
# * correct with $\theta_{acc}$: $K = P^-H^T/(HP^-H^T + r)$, $H = [1, 0]$, $x = x^- + K(\theta_{acc} - \theta^-)$.
#
# Write the prediction step (the correction is in `bootcamp.attitude.AngleKalman`).


# %% [exercise 4.5]
def kalman_predict(x, P, rate, dt, q_angle, q_bias):
    """Prediction step of the angle/bias Kalman filter. Returns (x, P) as new arrays."""
    ### BEGIN SOLUTION
    F = np.array([[1.0, -dt], [0.0, 1.0]])
    x_new = np.array([x[0] + (rate - x[1]) * dt, x[1]])
    P_new = F @ P @ F.T + np.diag([q_angle * dt, q_bias * dt])
    return x_new, P_new
    ### END SOLUTION


# %% [check 4.5]
for _ in range(10):
    x0, rate, step = rng.normal(size=2), rng.normal() * 20, 0.005
    M = rng.normal(size=(2, 2))
    P0 = M @ M.T + np.eye(2)
    kf = att.AngleKalman(q_angle=0.01, q_bias=0.003)
    kf.x, kf.P = x0.copy(), P0.copy()
    kf.predict(rate, step)
    x1, P1 = run("4.5", kalman_predict, x0, P0, rate, step, 0.01, 0.003)
    check_close("4.5", x1, kf.x, atol=1e-12, what="predicted state", quiet=True)
    check_close(
        "4.5", P1, kf.P, atol=1e-12, what="predicted covariance", quiet=True, hint="P = F P F^T + Q"
    )
check_true("4.5", True, "kalman_predict matches the reference on 10 random cases")

# %%
kf = att.AngleKalman(q_angle=0.02, q_bias=0.01, r_meas=40.0)
kf_est, kf_bias = [], []
for g, a in zip(log.gyro, log.accel_angle):
    kf_est.append(kf.update(g, a, dt))
    kf_bias.append(kf.bias)
f = att.ComplementaryFilter(att.alpha_from_time_constant(0.5, dt))
cf_est = np.array([f.update(g, a, dt) for g, a in zip(log.gyro, log.accel_angle)])
sim_rig = rig.Rig(seed=5)  # same seed: the same gyro bias as the run above
fig, axs = plt.subplots(1, 2, figsize=(10, 3.2))
axs[0].plot(
    log.t,
    np.array(kf_est) - log.theta,
    color=SERIES[2],
    lw=1,
    label=f"Kalman (RMS {rms((np.array(kf_est) - log.theta)[log.t > 2]):.2f} deg)",
)
axs[0].plot(
    log.t,
    cf_est - log.theta,
    color=SERIES[1],
    lw=1,
    label=f"complementary (RMS {rms((cf_est - log.theta)[log.t > 2]):.2f} deg)",
)
axs[0].set(xlabel="time (s)", ylabel="error (deg)", title="Estimation error")
axs[1].plot(log.t, kf_bias, color=SERIES[2], label="Kalman bias estimate")
axs[1].axhline(sim_rig.gyro_bias, **REF, label=f"true bias {sim_rig.gyro_bias:.2f} deg/s")
axs[1].set(xlabel="time (s)", ylabel="deg/s", title="The Kalman filter learns the gyro bias")
axs[0].legend()
axs[1].legend()
save(fig, "s04_kalman")

# %% [markdown]
# ## 6. Cascaded control: a rate loop inside an angle loop
#
# Every flight controller (Betaflight, PX4, ArduPilot) nests two loops per axis:
#
# * the **inner rate loop** compares the gyro rate with a rate setpoint and drives the
#   motors (PID, fast: it sees the cleanest, least-delayed signal);
# * the **outer angle loop** compares the estimated angle with the angle setpoint and
#   produces the rate setpoint (often P only, ~3-10x slower than the inner loop).
#
# Why: a disturbance torque shows up in the gyro immediately, so the inner loop fights it
# before the angle has moved much; and the rate limit of the outer loop gives a natural
# speed limit. The cost: two loops to tune, inner first. On the rig, `s04_rig_cascaded`
# does exactly this with two easyPID instances. The gyro bias is measured at boot, so
# the simulation below assumes a small residual bias.

# %%
SENS = rig.SensorModel(gyro_bias_dps=0.2)  # after the start-up bias calibration


def make_cascade(akp, rkp, rki, rkd, source="imu", tau=0.5):
    outer = ep.PIDController(akp, 0.0, 0.0, -150, 150)
    inner = ep.PIDController(rkp, rki, rkd, -200, 200)
    inner.setDerivativeFilter(ep.FILTER_EMA, 0.7)
    outer.begin()
    inner.begin()
    f = att.ComplementaryFilter(0.0)

    def ctrl(t, sp, m, dt):
        f.alpha = att.alpha_from_time_constant(tau, dt)
        acc = math.degrees(math.atan2(m["accel_x"], m["accel_z"]))
        est = f.update(m["gyro"], acc, dt) if source == "imu" else m["pot"]
        return inner.update(outer.update(sp, est, dt * 1000), m["gyro"], dt * 1000)

    return ctrl


def make_single(kp, ki, kd):
    pid = ep.PIDController(kp, ki, kd, -200, 200)
    pid.setDerivativeFilter(ep.FILTER_EMA, 0.6)
    pid.begin()
    return lambda t, sp, m, dt: pid.update(sp, m["pot"], dt * 1000)


def tap(t):  # a 0.3 N m push for 50 ms at t = 4 s: someone flicks the beam
    return 0.3 if 4.0 <= t < 4.05 else 0.0


def scenario(ctrl, seed=1):
    return rig.simulate_rig(
        ctrl,
        7.0,
        setpoint=lambda t: 10.0 if t >= 1.0 else 0.0,
        sensors=SENS,
        seed=seed,
        disturbance=tap,
    )


# %% [markdown]
# Specification for the cascade **using the IMU estimate** (not the pot) for the angle:
#
# | | limit |
# |---|---|
# | overshoot, 0 → 10° step at t = 1 s | < 10 % |
# | settling into ±1° | < 1.5 s |
# | peak deviation after the tap at t = 4 s | < 4° |
#
# Tune the inner loop first (`rkp`, then a little `rki`, `rkd` small), then `akp`.


# %% [exercise 4.6]
def cascade_gains():
    """Return (akp, rkp, rki, rkd)."""
    ### BEGIN SOLUTION
    # inner loop stiff (rkp 3) with a little rki and rkd; outer loop P at 4 1/s
    return 4.0, 3.0, 1.0, 0.05
    ### END SOLUTION


# %% [check 4.6]
g = run("4.6", cascade_gains)
res = scenario(make_cascade(*g))
m = (res.t >= 1.0) & (res.t < 4.0)
info = step_info(res.t[m], res.theta[m], 10.0, y0=0.0)
out = np.nonzero(np.abs(res.theta[m] - 10.0) > 1.0)[0]
settle = res.t[m][out[-1] + 1] - 1.0 if len(out) else 0.0
peak = np.abs(res.theta[(res.t >= 4.0) & (res.t < 6.0)] - 10.0).max()
check_less("4.6", info.overshoot_pct, 10.0, "overshoot", " %")
check_less("4.6", settle, 1.5, "settling into +-1 deg", " s")
check_less(
    "4.6",
    peak,
    4.0,
    "peak deviation after the tap",
    " deg",
    hint="a stiffer inner loop (rkp) rejects the tap",
)
single = scenario(make_single(2.0, 1.5, 0.8))
fig, ax = plt.subplots(figsize=(9, 3.4))
ax.plot(
    single.t, single.theta, color=SERIES[1], label="single PID loop on the pot (session 3 gains)"
)
ax.plot(res.t, res.theta, color=SERIES[0], label=f"cascade on the IMU estimate, gains {g}")
ax.plot(res.t, res.setpoint, **REF, label="setpoint")
ax.axvline(4.0, color=SERIES[3], lw=1, ls=":", label="tap")
ax.set(
    xlabel="time (s)",
    ylabel="beam angle (deg)",
    title="Cascaded rate/angle control rejects the tap better",
)
ax.legend(fontsize=8)
save(fig, "s04_cascade")

# %% [markdown]
# ## 7. Where you mount the IMU matters
#
# An accelerometer $r$ metres from the pivot also feels the centripetal ($-r\dot\theta^2$)
# and tangential ($r\ddot\theta$) accelerations of the beam. The same run with the IMU
# 20 cm out on the arm:

# %%
quiet = dict(vib_accel_g=0.0, vib_gyro_dps=0.0, accel_noise_g=0.0)  # isolate the lever-arm effect


def square(t):
    return 10.0 if (t % 2.0) < 1.0 else -10.0


far = demo_run(sensors=rig.SensorModel(imu_offset=0.20, **quiet), seconds=6.0, setpoint=square)
near = demo_run(sensors=rig.SensorModel(imu_offset=0.0, **quiet), seconds=6.0, setpoint=square)
fig, ax = plt.subplots(figsize=(9, 3.0))
ax.plot(far.t, far.accel_angle - far.theta, color=SERIES[1], label="IMU 20 cm from the pivot")
ax.plot(near.t, near.accel_angle - near.theta, color=SERIES[0], label="IMU on the pivot axis")
ax.set(
    xlabel="time (s)",
    ylabel="accel-angle error (deg)",
    title="Fast +-10 deg steps, no vibration: mount the IMU on the axis",
)
ax.legend()
save(fig, "s04_imu_mounting")
print(
    f"peak accel-angle error: on axis {np.abs(near.accel_angle - near.theta).max():.2f} deg, 20 cm out {np.abs(far.accel_angle - far.theta).max():.1f} deg"
)

# %% [markdown]
# ### Reflection (not auto-checked)
#
# On a real quadrotor the IMU cannot be on every rotation axis, and the vehicle
# accelerates for long periods (a turn, a climb). Why does a complementary filter on the
# accelerometer then give a wrong attitude, and what does a flight controller do about it?
#
# <!-- BEGIN SOLUTION -->
# The accelerometer measures specific force, gravity *minus* acceleration. In a sustained
# coordinated turn or a long acceleration the specific force is not vertical, so
# $\operatorname{atan2}$ of it is not the tilt; a slow filter averages a biased signal and
# converges to the wrong attitude. Flight controllers lower the accelerometer weight when
# $|a|$ differs from 1 g or when the vehicle is manoeuvring, compensate the known
# acceleration from a velocity estimate (GNSS, optical flow, motion capture -- the rotorfault
# study fuses motion-capture attitude), and rely on the gyro for the short term. The IMU is
# mounted near the centre of mass to minimise the lever-arm terms, and vibration is isolated
# mechanically and filtered.
# <!-- END SOLUTION -->
#
# **Summary.** Gyro for the short term, accelerometer for the long term, a filter to
# blend them; then a fast rate loop on the gyro inside a slower angle loop. That is a
# flight controller's attitude stack, and the rig now runs one.
