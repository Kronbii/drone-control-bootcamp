# Quiz answer keys

Worked answers for the seven quizzes in `sessions/*/quiz.md`. Quizzes 1-6 are formative
(self-marked); quiz 7 counts towards the certificate (≥ 6/10).

## Quiz 1 · Feedback and PID

1. $y(0.5) = 2(1 - e^{-1}) = 1.26$; it settles to $Ku = 2$.
2. Loop gain $L = KK_p = 9$. (a) $y_\infty = \frac{9}{10}\cdot 10 = 9$. (b) $\tau_{cl} = 0.5/10 = 0.05$ s.
3. (a) True: $e_{ss} = r/(1 + KK_p)$ falls as $K_p$ rises (as long as nothing saturates).
   (b) True: in steady state the integrator input, the error, must be zero.
   (c) False: a first-order plant has nothing for D to damp; speed comes from $K_p$ (within
   the actuator limits) and D mainly adds noise.
4. (b) 20 ms.
5. There is no previous error on the first call; differencing against 0 would give
   $e/\Delta t$, a large spike exactly when the error is largest (the "derivative kick").
6. $\omega_{cl} = 2\sqrt4 = 4$; $0.7 = (0.8 + 4K_d)/8 \Rightarrow K_d = 1.2$.
7. $K_p e = 20\cdot 2.5 = 50$ V is requested but the pin gives at most 5 V: the output saturates,
   the loop is effectively open, and the capacitor charges towards 5 V with $\tau = RC = 1$ s
   until the error is below $5/K_p = 0.25$ V. The formula assumes no saturation.
8. Gives: rejection of disturbances and insensitivity to model errors. Costs: a sensor, and
   the possibility of instability (the controller reacts to its own past actions).

## Quiz 2 · Tuning and real-world PID

1. $a = e^{-0.2} = 0.819$; $b = 2(1 - 0.819) = 0.363$.
2. $360\cdot 3\cdot\frac{1/30}{2} = 18°$.
3. Integrator windup. `setAntiWindup(ANTIWINDUP_CLAMP)` (the default) or `ANTIWINDUP_BACKCALC`;
   CLAMP undoes a sample's integration when the output is saturated and the error pushes
   further into the limit.
4. $f_c = -\ln 0.9/(2\pi\cdot 0.005) = 3.35$ Hz. At 100 Hz: $\alpha = e^{-2\pi\cdot3.35\cdot0.01} = 0.81$ (= 0.9²).
5. easyPID differentiates the error, which jumps with the setpoint: one sample of
   $K_d\Delta r/\Delta t$. Ramp the setpoint (or filter it, or differentiate the measurement).
6. $K = 4/2 = 2$; $\tau = 1.5(3.0 - 1.2) = 2.7$ s; $L = 3.0 - 2.7 = 0.3$ s.
7. $K_u = 4\cdot2/(\pi\cdot0.1) = 25.5$. ZN: $K_p = 15.3$, $K_i = 1.2\cdot25.5/2.6 = 11.8$,
   $K_d = 0.075\cdot25.5\cdot2.6 = 5.0$.
8. The tuner averages cycles from the first relay switch on, while the limit cycle of this
   slow plant is still growing: early cycles are small and fast, so the amplitude is
   underestimated ($K_u$ too high) and the period too short. A second run starts with the
   limit cycle established and measures it correctly (within the describing-function error).

## Quiz 3 · The seesaw rig

1. $J\ddot\theta = l(T_1 - T_2) - mgh\sin\theta - b\dot\theta$: $J$ inertia about the pivot, $l$ pivot-to-motor
   distance, $h$ distance of the centre of gravity below the pivot, $b$ viscous friction.
2. $0.180\cdot9.81 = 1.77$ N. The beam's weight is a constant load that the tare removes; the
   moment balance about the pivot gives $T_1 l = \Delta F\,l$ for equal arms.
3. $k_u = 2\cdot0.25\cdot(1.6 + 2\cdot6.4\cdot0.35)/1000 = 3.04\times10^{-3}$ N m/µs, so 50 µs gives 0.15 N m.
4. Below the pivot (it swings like a pendulum): $h = J(2\pi/T)^2/(mg) = 0.0126\cdot1.496^2/(0.29\cdot9.81) = 9.9$ mm.
5. The rig is a lightly damped pendulum with a motor lag. P adds stiffness but no damping,
   and the lag adds phase; above about 0.8 µs/deg (model) the poles cross into the right
   half-plane. D adds damping (phase lead).
6. Clipping one motor shrinks $\text{us}_1 - \text{us}_2$, i.e. the torque, exactly when the controller
   asks for more. Shifting both keeps the torque and gives up a little collective thrust,
   which the rig does not need.
7. Any four: OFF→ON arming transition (switch left on at power-up); pot plausibility (loose
   wire); within ±20° to arm (start from a safe attitude); 3 s boot delay (ESC start-up);
   trip beyond ±28° for 300 ms (diverging controller); `x` stop command; throttle cap
   1600 µs (limits energy); 2 s ramp-up (no step to full base throttle); IMU failure disarm
   in the cascaded sketches (blind inner loop).
8. It must stop the motors even if the software has crashed, hung or is wrong; a physical
   break in the power path does not depend on any code.

## Quiz 4 · Rotations and attitude

1. $R_z(90°) = \begin{bmatrix}0&-1&0\\1&0&0\\0&0&1\end{bmatrix}$; $R_z(90°)(1,0,0)^T = (0,1,0)^T$: after a 90° yaw the
   body x axis points along world y (left).
2. No. $R_y(90°)e_1 = (0,0,-1)$ and $R_z(90°)$ leaves it there; but $R_z(90°)e_1 = (0,1,0)$ and
   $R_y(90°)$ leaves that there: $R_zR_ye_1 \ne R_yR_ze_1$.
3. Gimbal lock: roll and yaw axes coincide, only their sum or difference is defined and the
   Euler-angle rates become singular. Quaternions and matrices have no such singularity.
4. $q^* = (\cos45°, 0, 0, -\sin45°)$: a rotation by -90° about z, the inverse of $q$.
5. $\operatorname{atan2}(0.5, 0.866) = 30°$. Not while swinging: the accelerometer then also measures
   the beam's centripetal and tangential acceleration (unless it is on the axis) and vibration.
6. $\alpha = 0.5/0.505 = 0.990$; steady error $\approx b\tau = 2\cdot0.5 = 1°$.
7. It estimates the gyro bias (and chooses its gain from the noise statistics).
8. The inner rate loop uses the gyro, is the faster one and is tuned first. A disturbance
   torque shows up in the rate at once, so the inner loop counteracts it before an angle error
   has built up.

## Quiz 5 · Quadrotor dynamics and control

1. The thrust always acts along body z; a horizontal force requires tilting it.
2. Row 1: $[1, 1, 1, 1]$; row 2 ($\tau_x = \sum y_if_i$): $[-a, a, a, -a]$.
3. $\Omega_h = \sqrt{0.64\cdot9.81/(4\cdot8.96\times10^{-7})} = 1324$ rad/s; $T/W = 4\cdot5.6/(0.64\cdot9.81) = 3.57$.
4. Yaw: its authority is small ($\kappa$ = 1.5 cm vs a 7.8 cm lever for roll/pitch) and a slow
   unwanted yaw is harmless; roll/pitch keep the vehicle upright.
5. $\theta_d = 2/9.81 = 0.204$ rad = 11.7°. For $a_y = +2$: $\phi_d = -a_y/g = -0.204$ rad, a negative roll.
6. Position → velocity → attitude → rate. Each loop is several times faster than the one
   around it, so the outer loop can treat the inner one as ideal.
7. The rate loop. Lower its bandwidth: reduce `rate_kp` (and `rate_kd`), accepting slower
   attitude and position response.
8. The reaction torque of accelerating rotors ($s_iJ_r\dot\Omega_i$, plus rotor gyroscopics). A yaw
   command speeds up one spin direction and slows the other, so their reaction torques add
   about z, comparable to the small drag torque; a roll command changes both spin directions
   equally and they cancel.

## Quiz 6 · Faults and fault tolerance

1. $\eta = 0.3$.
2. With zero torque each rotor must give $mg/4$, so $\eta f_{max} \ge mg/4$, $\eta \ge 1/(T/W) = 0.4$.
3. The residual direction a fault on rotor $j$ produces ($d_j = -B_{:,j}f_j$). Projecting the
   (noise-normalised) residual on each normalised signature and picking the largest is a
   matched filter: the fault whose direction best explains the residual.
4. The delay grows (roughly linearly in $h$); the false-alarm rate falls (the average run length
   to a false alarm grows roughly exponentially in $h$).
5. A test designed for a small fault accumulates evidence of a large fault at the small
   design rate, so large faults are detected late (the study's ablation doubled the median
   delay); a bank approximates a test for an unknown magnitude.
6. Twice the command ($1/\hat\eta_1 = 2$).
7. Three thrusts cannot balance the remaining rotors' yaw drag torque and the roll/pitch
   torques while carrying the weight with zero yaw rate. Spinning about a tilted primary axis
   creates a gyroscopic balance; the controller keeps the *direction* of that axis (a reduced
   attitude) and the thrust, hence position, and gives up heading.
8. Any two, with a measurement: realistic narrow-band vibration (blackbox gyro spectra on a
   real frame); actuator delay and motor time constant (RPM telemetry, test stand);
   rotational damping and inertia (spin-down and swing tests); motion capture instead of
   GNSS (fly with the real estimator); a clean step fault (test with a weakened prop).

## Quiz 7 · Final concept check (certificate: ≥ 6/10)

1. $KK_p/(1 + KK_p) = 8/9 = 0.89$.
2. While saturated, more output does nothing but the integrator keeps growing; when the
   error reverses it must unwind first (large, slow overshoot). Default `ANTIWINDUP_CLAMP`:
   conditional integration, undoing a sample's accumulation when the output is saturated and
   the error pushes further into saturation.
3. The argument is in milliseconds, so 0.01 means 10 µs: the integral grows 1000x too slowly
   and the derivative is 1000x too large (noise and spikes dominate).
4. Lightly damped pendulum plus motor lag: P adds stiffness, not damping. Rate feedback (D,
   later the gyro-based inner loop) added the damping.
5. Accelerometer: measures specific force, wrong during accelerations (turns, climbs) and
   swamped by vibration. Gyro: integrating its bias drifts without bound.
6. The rate loop is faster and tuned first.
7. Yaw torque.
8. No: $\eta_{min} = 1/3 = 0.33 > 0.3$. It could hold position only by giving up heading
   (yaw-relaxed control).
9. It accumulates the log-likelihood ratio of "fault" against "no fault", reset at zero. The
   study sets thresholds from fault-free calibration flights (2x the largest statistic seen,
   floored at the evidence of a design-size fault in 20 ms).
10. Open answer; it must name a simulation result and a concrete measurement (for example:
    simulated rig gains → measured thrust curve and motor lag; complementary-filter τ →
    vibration spectrum on the rig; complete-loss recovery in rotorfault → actuator delay and
    motor time constant on a test stand).
