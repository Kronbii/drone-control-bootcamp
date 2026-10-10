# Lab 5 · The simulation is the lab: break the quadrotor on purpose

*Teams of 2-3, laptops only. Work in a new notebook (`notebooks/my_lab5.ipynb`) or a script;
everything you need is in `bootcamp.quad`. Keep every experiment's plot: the deliverable
is a one-page "failure catalogue".*

**You hand in:** for each experiment below, one plot and two sentences: *what broke, and
which loop or assumption it broke*.

```python
import numpy as np
from bootcamp import quad

ctrl = quad.CascadedController()  # nominal gains and model
ref = quad.waypoints([(0, 0, 1), (2, 0, 1), (2, 2, 1.5), (0, 0, 1)], dwell=4.0)
log = quad.simulate_quad(ctrl, ref, 16.0)  # 500 Hz control, RK4 physics
```

Create a **new** `CascadedController` for every run (it holds integrator state).

## 1. Baseline (10 min)

Fly the course above. Plot $x, y, z$ against the reference, the roll/pitch angles and the
four rotor thrusts. What is the largest tilt? How close do the thrusts come to $f_{max}$?

## 2. Slow motors (10 min)

`simulate_quad(..., motor_scale=k)` multiplies the *true* motor time constant. Find the
smallest $k$ at which the attitude oscillates visibly, and the one at which it crashes. Then
lower `rate_kp` (e.g. `quad.CascadeGains(rate_kp=(12, 12, 6))`) and see how much lag the
vehicle now tolerates -- and what it costs in tracking.

## 3. A heavier vehicle than the controller thinks (10 min)

`simulate_quad(ctrl, ref, 16.0, params=quad.QuadParams(mass=0.8))` makes the true vehicle
25 % heavier while the controller still assumes 0.638 kg. Which integrator saves you, and
how long does it take? Now try `quad.CascadeGains(vel_ki=(1, 1, 0))` (no vertical integrator).

## 4. Noisy gyro (10 min)

`simulate_quad(..., gyro_noise=0.05)` adds 0.05 rad/s of white noise to the rate the
controller sees (about 3 deg/s, typical after filtering on a vibrating frame). Plot the
rotor thrusts. Compare `rate_d_alpha=0.0` and `0.9` (the easyPID EMA filter on the rate
D term): what happens to the motor commands, and to the tracking?

## 5. Less thrust margin (10 min)

Give the vehicle smaller motors: `P = quad.QuadParams(omega_max=1700)` has a thrust-to-weight
ratio of 1.65 instead of 3.58. Use the same `P` for the controller and the vehicle
(`quad.CascadedController(params=P, gains=quad.CascadeGains(v_max=6.0, max_tilt_deg=50))`
and `simulate_quad(..., params=P)`), and command a 4 m step in x together with a yaw change
of 1 rad (write a reference function returning `quad.Reference(p=..., yaw=...)`).
Compare with the nominal vehicle: how often do the rotors hit $f_{max}$, how long does the
step take, and does yaw still track? Then compute $1/(T/W)$ for both vehicles -- session 6
shows why that number decides whether a quad can survive a weak rotor.

## 6. The research model (if time allows)

From a terminal:

```bash
rotorfault simulate --traj fig8 --loe 0 --duration 10 --plot logs/fig8.png
rotorfault simulate --traj hover --rotor 2 --loe 0.5 --baseline passive --plot logs/loe50.png
```

The study model adds sensor noise, a 10 ms motion-capture delay, parameter spread and
turbulence. Compare its hover position error with our noise-free model: which is closer
to what you would expect on a real 5-inch quad, and why? (This is where session 6 starts.)
