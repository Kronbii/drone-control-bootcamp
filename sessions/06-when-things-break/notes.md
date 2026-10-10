# Session 6 notes · Fault detection and fault-tolerant control

Numbers quoted below come from the notebook runs or from the rotorfault study's own
README. They are **simulation results with assumed parameters**; nothing here has been
flown.

## 1. Faults as loss of effectiveness

A chipped propeller, a prop strike, a loose motor or a failing ESC channel all mean that
rotor $j$ delivers less thrust than commanded. The simplest useful model is a
multiplicative **loss of effectiveness** (LOE): the vehicle produces

$$\begin{bmatrix}T\\\tau\end{bmatrix} = B\,\mathrm{diag}(\eta)\,f, \qquad \eta_j\in[0, 1],$$

with $\eta_j = 1$ healthy, $\eta_j = 0$ a complete loss; "60 % LOE" means $\eta_j = 0.4$. In the
study the fault is a step at a known-to-nobody onset time, on one rotor at a time.

## 2. Hover with a weak rotor: the hard limit

With zero torque, $B$ invertible means each rotor must deliver exactly $mg/4$, so rotor $j$
needs $\eta_jf_{max} \ge mg/4$:

$$\eta_j \ge \eta_{min} = \frac{mg}{4f_{max}} = \frac{1}{T/W}.$$

For the study's quad ($T/W = 3.58$), $\eta_{min} = 0.28$: beyond about 72 % LOE no controller
can hold the vehicle level **and** its heading. A higher thrust-to-weight ratio buys fault
tolerance; a heavily loaded quad has almost none. (The study switches to yaw-relaxed
control below an estimated $\eta = 0.35$, leaving margin for attitude control.)

**Passive behaviour.** A controller with integral action absorbs a moderate loss after a
transient: the attitude integrators find the new torque balance. Its peak deviation grows
with the loss (study, E1: about 46 cm at 60 % LOE versus 4 cm for the active pipeline), and
a complete loss flips it within about 0.3 s.

## 3. Residuals and isolation

A **residual** is "what happened" minus "what the healthy model predicts". The study builds
four, from the differentiated gyro, the accelerometer and the motor commands passed through
a motor model: $y = B\,\mathrm{diag}(f)\,\eta$ (rotational dynamics plus the vertical force
balance), so with healthy prediction $Bf$ the residual of a loss $\delta_j$ on rotor $j$ is

$$r = -\delta_j\,B_{:,j}f_j + \text{noise} = \delta_j\,d_j + \text{noise}.$$

Each rotor has a known **signature** direction $d_j$. Normalise each row by its noise level,
project, and pick the best match:

$$t_j = \frac{d_j^Tr}{\lVert d_j\rVert}, \qquad \hat j = \arg\max_j t_j .$$

In practice the residual is coloured by turbulence and model error; the study filters all
signals through the same 15 Hz state-variable filter (so the regression stays exact) and
widens the noise scale when manoeuvres excite model uncertainty.

## 4. Detection: CUSUM

Deciding *when* the mean of a noisy statistic has shifted is a sequential test. Page's
**CUSUM** for a shift from 0 to $\mu_1$ in Gaussian noise of standard deviation $\sigma$:

$$g_k = \max\Big(0,\ g_{k-1} + \frac{\mu_1}{\sigma^2}\big(r_k - \tfrac{\mu_1}{2}\big)\Big), \qquad \text{alarm at } g_k > h .$$

Each step adds the log-likelihood ratio of "shifted" versus "not shifted"; the $\max(0,\cdot)$
forgets evidence against a fault. Raising $h$ lowers the false-alarm rate (the average run
length to a false alarm grows roughly exponentially in $h$) and lengthens the detection
delay (roughly linearly). The study:

- runs a **bank** of CUSUMs per rotor designed for 15, 30 and 60 % LOE (a single small-fault
  test detects large faults slowly: its ablation doubled the median delay);
- **calibrates** thresholds on fault-free flights with disjoint seeds (threshold = 2x the
  largest statistic seen) because formulas assume white noise;
- reports detection of all 200 faults of 20-90 % LOE, median delay 30 ms, and no false
  alarm in 32 minutes of fault-free simulated flight.

## 5. Reconfiguration

With rotor $j$ isolated and $\hat\eta$ estimated (the study uses a Kalman filter on $\eta$ with a
random-walk model), allocate through the damaged vehicle's matrix:

$$f = \big(B\,\mathrm{diag}(\hat\eta)\big)^{-1}\begin{bmatrix}T\\\tau\end{bmatrix} \quad(\text{with the saturation priorities of session 5}).$$

Rotor $j$ is simply asked for $1/\hat\eta_j$ times more. This works while $\eta_j > \eta_{min}$.
The supervisor logic: NOMINAL → PARTIAL on an alarm → RELAXED if $\hat\eta_j < 0.35$ for 30 ms.

## 6. Complete loss: give up yaw

With one rotor dead, three thrusts cannot balance three torques *and* carry the weight
while yaw is held: the drag torques of the remaining rotors no longer cancel. Mueller and
D'Andrea (ICRA 2014) showed that the vehicle can still hold **position** if it is allowed to
**spin** about a body-fixed *primary axis* $\bar n$:

- the three working rotors produce a thrust along body z and torques such that, together
  with the gyroscopic term $\omega\times J\omega$, the rotor angular momentum and the
  rotational damping, the net torque is zero at a constant spin $\bar\omega \parallel \bar n$;
- $\bar n$ is tilted from body z, so the thrust direction cones around $\bar n$; on average it
  points along $\bar n$, which the controller keeps vertical: the weight is carried by
  $T\bar n_z$;
- control acts on the **reduced attitude** (the direction of $\bar n$, two degrees of
  freedom) and the thrust, never on the heading.

The study solves the equilibrium numerically (for rotor 1 lost and $\bar n$ tilted 15°:
three thrusts of about 2.3, 1.3 and 2.9 N, a spin of 24 rad/s -- inside the 2000 deg/s range
of FPV gyros), linearises the reduced attitude dynamics on a grid of yaw rates and
gain-schedules an LQR. In 32/32 simulated hover runs it recovered a complete loss that
the passive controller lost every time, with a median peak deviation of 38 cm.

## 7. Reading the limits

From the study's own section 6: simulation only, assumed parameters (rotational damping,
which sets the spin rate, is the least certain), white-noise vibration (real 5-inch frames
have large narrow-band vibration), 10 ms motion capture rather than GNSS, one clean step
fault at a time, a fault-free initial hover to learn the baseline, and a robustness sweep in
which complete-loss recovery **failed** at 24 ms of actuator delay (2 ms assumed), at an
80 ms motor time constant (25 ms assumed) and at 3x the default turbulence. A good study
tells you where it breaks; read that part first.

## Common mistakes

- Confusing LOE (fraction lost) with $\eta$ (fraction left).
- Isolating by the largest raw residual component instead of the signature projection.
- Setting a CUSUM threshold from one fault-free run.
- Re-allocating with the *true* $\eta$ in your head: the controller only has $\hat\eta$, late and noisy.
- Reading "recovered in simulation" as "will recover in flight".

## Further reading

- M. W. Mueller, R. D'Andrea, "Stability and control of a quadrocopter despite the complete
  loss of one, two, or three propellers", ICRA 2014.
- E. S. Page, "Continuous inspection schemes", *Biometrika*, 41, 1954; M. Basseville,
  I. Nikiforov, *Detection of Abrupt Changes*, 1993 (free online).
- The rotorfault README and paper (`paper/main.pdf` in its repository).
