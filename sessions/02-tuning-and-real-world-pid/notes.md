# Session 2 notes · Tuning and real-world PID

## 1. Sampling and discretisation

A digital controller reads the sensor every $T_s$ seconds and holds its output constant
until the next sample (a **zero-order hold**, ZOH). For $\tau\dot y + y = Ku$ with $u$
constant over a sample, the exact discrete model is

$$y_{k+1} = a\,y_k + b\,u_k, \qquad a = e^{-T_s/\tau}, \qquad b = K(1-a).$$

For a general linear plant $\dot x = Ax + Bu$: $A_d = e^{AT_s}$, $B_d = \int_0^{T_s} e^{A\sigma}d\sigma\,B$,
computed with one matrix exponential of the block matrix $\begin{bmatrix}A & B\\0 & 0\end{bmatrix}T_s$.

**What sampling costs.** On average the held output is $T_s/2$ late. A pure delay $T_d$ has
gain 1 and phase $-360 f T_d$ degrees at frequency $f$. Phase lag reduces the phase margin,
so the same gains become less damped as $T_s$ grows. Rules of thumb:

- sample at least **10-20x** faster than the closed-loop bandwidth;
- keep the loop period **fixed** (all lab sketches gate the loop on `millis()`/`micros()`)
  and pass the measured period to easyPID: `pid.update(sp, y, dtMs)`;
- sensor and actuator delays (an ESC frame, a filter, a 10 ms motion-capture pipeline) add
  to this, and in session 5 they set how fast the innermost loop may be.

## 2. Saturation and integrator windup

Every actuator has limits. While the output is clamped, the plant receives less than the
controller asks for, but an unprotected integrator keeps accumulating error. When the error
finally reverses, the integrator holds a large value that must unwind before the output
leaves the limit: long, large overshoot. Symptoms in a log: the output sits on a rail long
after the error changed sign; the I term is much larger than the output range.

easyPID's modes (`pid.setAntiWindup(...)`):

| mode | rule | when |
|---|---|---|
| `ANTIWINDUP_CLAMP` (**default**) | conditional integration: if this sample's raw output is above `outMax` with $e > 0$ (or below `outMin` with $e < 0$), undo this sample's integration; integration that drives the output back out of saturation is allowed | almost always |
| `ANTIWINDUP_BACKCALC` | add $(u_{clamped} - u_{raw})/K_i \cdot \Delta t$ to the integrator while saturated | large, frequent saturation; smooth recovery |
| `ANTIWINDUP_NONE` | no protection | demonstrations only |

`setIntegralLimits(min, max)` additionally bounds the raw error-time accumulator (not the
$K_i$-scaled term: the output contribution is $K_i \times$ limit).

## 3. Derivative noise and the EMA filter

Differencing a noisy measurement amplifies noise by $1/\Delta t$: a 1-count flicker of a
10-bit ADC at 100 Hz looks like a rate of 100 counts/s. easyPID can low-pass the
derivative: `pid.setDerivativeFilter(FILTER_EMA, alpha)`,

$$d^f_k = \alpha\, d^f_{k-1} + (1-\alpha)\, d_k, \qquad 0 \le \alpha \le 0.999 .$$

This first-order filter has time constant and cut-off

$$\tau_f = -\frac{T_s}{\ln\alpha}, \qquad f_c = -\frac{\ln\alpha}{2\pi T_s}, \qquad \alpha = e^{-2\pi f_c T_s}.$$

| $\alpha$ | $f_c$ at 100 Hz | $f_c$ at 200 Hz |
|---|---|---|
| 0.5 | 11.0 Hz | 22.1 Hz |
| 0.7 | 5.7 Hz | 11.4 Hz |
| 0.9 | 1.7 Hz | 3.4 Hz |

The same `alpha` is a different filter at a different loop rate. Place $f_c$ a few times
above the loop bandwidth: lower and the filter's own lag destabilises the loop.

## 4. Setpoint jumps: the derivative kick

easyPID differentiates the **error** $e = r - y$. A step $\Delta r$ in the setpoint produces
one sample with $d = K_d\Delta r/\Delta t$ (easyPID suppresses only the first sample after
`begin()`/`reset()`). Remedies: ramp the setpoint (`s02_rc_tuning` command `ramp`), or
filter it, or (in your own code) differentiate the measurement instead of the error.

## 5. Measuring a step response

Definitions used everywhere in this course (`bootcamp.metrics.step_info`):

- **rise time**: 10 % to 90 % of the step;
- **overshoot**: $100\,(y_{peak} - r)/(r - y_0)$ %, zero if $y$ never passes $r$;
- **settling time**: first time after which $|y - r|$ stays within a band (2 % of the step,
  or an absolute band such as ±1° on the rig);
- **steady-state error**: $r$ minus the mean of the last 10 % of samples;
- **IAE**: $\int|r - y|\,dt$.

Always log to a file and compute these from data; do not read them off a scrolling plot.

## 6. Identification: first order plus dead time

Many plants look like $G(s) = K e^{-Ls}/(\tau s + 1)$ (FOPDT) in an open-loop step test.
**Two-point method**: with $t_{28}$ and $t_{63}$ the times at which the response covers
28.3 % and 63.2 % of its final change,

$$\tau = 1.5\,(t_{63} - t_{28}), \qquad L = t_{63} - \tau, \qquad K = \Delta y/\Delta u .$$

The 3-stage RC ladder of lab 2 (exact transfer function $1/(1 + 6sT + 5s^2T^2 + s^3T^3)$,
$T = RC = 1$ s) fits $K = 1$, $\tau \approx 5.1$ s, $L \approx 0.95$ s, and the rule below gives
$K_p \approx 6.4$, $K_i \approx 3.4$, $K_d \approx 3.1$.

**Reaction-curve (open-loop Ziegler-Nichols) rule**, as in easyPID's tuning guide:
$K_p = 1.2\tau/(KL)$, $K_i = K_p/(2L)$, $K_d = 0.5K_pL$. It assumes $L > 0$ and works best for
$\tau/L$ between about 1 and 10; it is aggressive (quarter-amplitude decay).

## 7. Relay autotuning with easyPID's `PIDTuner`

Replace the controller by a relay of amplitude $d$ around an operating point. A plant with
enough phase lag oscillates at its **ultimate frequency**, where its phase is -180°. With
$a$ the half peak-to-peak oscillation and $h$ the relay hysteresis (easyPID's noise band),
the describing function of the relay gives

$$K_u = \frac{4d}{\pi\sqrt{a^2 - h^2}}, \qquad P_u = \text{oscillation period}.$$

easyPID then applies one of four rules:

| rule (`TuningRule`) | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|
| `TUNING_ZIEGLER_NICHOLS` | $0.6K_u$ | $1.2K_u/P_u$ | $0.075K_uP_u$ |
| `TUNING_TYREUS_LUYBEN` | $K_u/2.2$ | $K_p/(2.2P_u)$ | $K_pP_u/6.3$ |
| `TUNING_PESSEN` | $0.7K_u$ | $1.75K_u/P_u$ | $0.105K_uP_u$ |
| `TUNING_NO_OVERSHOOT` | $0.2K_u$ | $0.4K_u/P_u$ | $0.066K_uP_u$ |

Usage (easyPID 1.1.0):

```cpp
PIDController pid(1.0, 0.0, 0.0, 0.0, 5.0);
PIDTuner tuner(pid);
tuner.start(setpoint, relayAmplitude, noiseBand, outputBias);  // bias: unipolar actuators
// in loop(), while tuner.getState() == TUNER_RELAY_STEP:
float u = tuner.update(measurement);
// afterwards: tuner.isComplete(), tuner.getUltimateGain(), tuner.getUltimatePeriod(),
//             tuner.getTunings(kp, ki, kd, TUNING_TYREUS_LUYBEN) or tuner.applyTunings(rule)
```

**What testing against a known answer shows.** The RC ladder's exact ultimate point is
$K_u = 29$, $\omega_u = \sqrt6/RC$ ($P_u = 2.57$ s for $RC = 1$ s). In simulation with the easyPID
tuner (the Python port, which the course tests against the C++ sources):

1. A first run from rest gives $K_u \approx 42$, $P_u \approx 2.0$ s. easyPID averages cycles from
   the first relay switch, while this slow plant's oscillation is still growing.
2. A second run started immediately afterwards (plant already oscillating) gives
   $K_u \approx 28$, $P_u \approx 2.64$ s -- within a few percent. **The lab sketch therefore
   runs the tuner twice and reports the second run.**
3. The noise band is relay hysteresis: it shifts the oscillation to where the plant phase
   is $-180° + \arcsin(h/a)$. On this plant $h = 5$ mV gives $K_u \approx 25$, $h = 20$ mV about 21.
   Use the largest relay amplitude the plant tolerates and the smallest noise band that
   stops chatter; treat the result as a starting point.

Never relay-tune a plant that cannot tolerate a sustained oscillation (the seesaw rig, a
flying drone): use a model and a gentle manual procedure instead.

## 8. Manual tuning procedure (when there is no model)

1. $K_i = K_d = 0$. Raise $K_p$ until the response is fast with slight oscillation; back off to 50-70 %.
2. If the plant overshoots or rings, add $K_d$ (start near $K_p\cdot T/10$ for a dominant time
   constant $T$); add the EMA filter if the output gets noisy.
3. Add $K_i$ to remove the offset; increase until the offset disappears in an acceptable
   time without extra overshoot.
4. Check with a setpoint step *and* a disturbance; check saturation (windup) with a large step.
5. Change one gain at a time, by about 25 %, and log every attempt.

## Further reading

- K. J. Åström, T. Hägglund, *Advanced PID Control*, ISA, 2006 (relay autotuning, anti-windup).
- easyPID `docs/tuning_guide.md` and `src/PIDTuner.cpp` (github.com/Kronbii/easyPID).
