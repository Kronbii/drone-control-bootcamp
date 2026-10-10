# Lab 6 · Fault experiments: in the research simulator, and on the rig

*Teams of 2-3. Part A is simulation (laptops). Part B is optional and props-on: cage
closed, checklist, roles ([safety.md](../../hardware/rig/safety.md)).*

**You hand in:** the table and plot of part A1-A2, your false-alarm count (A3), the result
of the robustness probe (A4), and -- if you did part B -- the detection delay of your own
CUSUM on the rig log.

## Part A · rotorfault experiments (40 min)

Use `bootcamp.fault.run_case` (one closed-loop simulation, about 1-2 s of computer time):

```python
from bootcamp import fault

res, summary = fault.run_case(
    loe=0.6, rotor=2, baseline="estimated", duration=5.0, onset=2.0, seed=0
)
print(summary)  # lost?, detection delay, isolated rotor, max deviation, switch to relaxed, spin
```

**A1. Loss sweep.** For LOE = 0.2, 0.4, 0.6, 0.8, 1.0 on rotor 2, run `passive` and `estimated`
with seeds 0 and 1. Tabulate max deviation (cm) and lost/not lost. Compare with table E1 of
the rotorfault README (8 seeds per cell, rotor 1): same trend?

**A2. Detection delay.** From the `estimated` runs, plot detection delay against LOE. Why do
small faults take longer? (Think about the CUSUM drift $\mu_1(r - \mu_1/2)$.)

**A3. False alarms.** Five fault-free runs (`loe=0.0`) with seeds 10-14, 6 s each. Any alarm
in a fault-free run is a false alarm: count the runs in which `res.alarm_time` is finite
(`math.isfinite(res.alarm_time)`). How many? What does that number tell you, and what
does it *not* tell you?

**A4. Probe a limit.** The study's complete-loss recovery failed when the true motors were
3.2x slower than the controller assumed. Reproduce one point:

```python
from rotorfault.fdi import FDIConfig
from rotorfault.scenarios import make_scenario
from rotorfault.sim import simulate

for k in (1.0, 2.4, 3.2):
    sc = make_scenario("hover", 5.0, rotor=0, magnitude=1.0, onset=2.0, true_scale={"tau_motor": k})
    r = simulate(sc, "estimated", 3, FDIConfig())
    print(k, "lost" if r.crashed else "recovered", r.crash_reason)
```

One seed is an anecdote, not a result: which of your findings would you trust, and how
many seeds would you run before writing it in a report?

## Part B · A fault on the rig, detected with your own CUSUM (optional, 30 min)

The rig sketches have a fault-injection command: `fault 0.7` scales motor 2's throttle above
idle by 0.7 -- a software stand-in for a damaged propeller.

1. Upload `s04_rig_cascaded` with your session-4 gains, `src pot`, arm, hold `sp 0`.
2. Start the logger (it restarts the sketch: re-arm), let the beam settle, then type
   `fault 0.7` into the logger; about 15 s later `fault off`; stop the logger.
3. Plot `u`: before the fault it hovers around 0; after it, the controller needs a steady
   negative differential to hold the beam level (more on the weak motor-2 side means
   *negative* `u`, since motor 2 = base − u). **That steady shift is a residual**: "what the
   controller has to do" minus "what a healthy rig needs".
4. Estimate $\sigma$ of `u` over the first 8 s and the shift $\mu_1$ after the fault. Run your
   `my_cusum` from notebook exercise 6.4 on `-u` (so the shift is positive), choose $h$ so
   that the pre-fault part never alarms, and report the detection delay. Its sign isolates
   the weak motor.
5. Discuss: why is this residual only usable while the setpoint is constant? (Hint: what
   else makes `u` move?)

## Checklist

- [ ] part A table, plot, false-alarm count, robustness probe
- [ ] (part B) log, CUSUM delay, `fault off` sent, rig disarmed and props removed
