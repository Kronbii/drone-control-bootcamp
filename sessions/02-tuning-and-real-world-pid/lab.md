# Lab 2 · Identify, autotune and break a PID loop on purpose

*About 45 minutes, teams of 2-3. 5 V only. Capacitors: stripe to GND.*

**Plant:** a 3-stage RC ladder (each stage 10 kΩ + 100 µF), measured on A2:

```
 D5 --[10k]--+--[10k]--+--[10k]--+-- A2
             |         |         |
          [100uF]   [100uF]   [100uF]     (A0 and A1 may also be wired to the
             |         |         |          first two nodes; "tap 0/1/2" selects)
 GND --------+---------+---------+
```

Its exact transfer function is $1/(1 + 6sT + 5s^2T^2 + s^3T^3)$ with $T = RC = 1$ s: ultimate
gain 29, ultimate period 2.57 s, and an FOPDT fit of about $K = 1$, $\tau = 5.1$ s, $L = 0.95$ s.
Your capacitors have tolerances, so your numbers will differ; the point is to measure them.

**Sketch:** `arduino/s02_rc_tuning`. Commands are listed at the top of the file; every
row of output is `t_ms,sp,y,u,p,i,d,mode`. Log each part with
`python tools/serial_logger.py --port <port> --seconds <s> logs/lab2_<part>.csv`
(close the Serial Plotter first) and **type the commands into the logger's terminal**.
Starting the logger restarts the sketch (default gains, 0 V output), so start it before
each part and give the plant time to settle. Analyse in notebook 2 with
`bootcamp.logs.read_log`.

**You hand in:** a table with your FOPDT fit, your relay $K_u$/$P_u$ (both runs), the gains of
two rules, and one plot of the windup experiment.

## Part A · Open-loop step test and the reaction curve (12 min)

1. Start the logger for 60 s (`--seconds 60`), type `open`, `u 1`, and wait ~20 s until A2
   is steady at about 1 V.
2. Type `steptest 1 3 35` (2 s at 1 V, then 3 V for 35 s).
3. In the notebook: `fit_fopdt_two_point(t, y, du=2.0)` on the part after the step
   (subtract the step time), then `ziegler_nichols_open_loop(...)`.
4. Try those gains: `kp ... ki ... kd ...`, `pid`, `sp 2.5`. Comment on the overshoot.

## Part B · Relay autotune with easyPID's PIDTuner (15 min)

1. Start the logger for 90 s; type `pid`, `sp 2.5` and wait until steady (the relay is
   centred on the setpoint, and the sketch passes the setpoint as easyPID's `outputBias`,
   because the plant's static gain is 1).
2. Type `tune 2.0 0.005` (relay ±2 V, noise band 5 mV).
   The sketch runs the tuner **twice**: pass 1 lets the oscillation grow and settle, pass 2
   measures it (see notes §7 for why). It prints $K_u$, $P_u$ and the gains of the four rules.
3. From the log, measure the period of the oscillation yourself (time between rising
   crossings of the setpoint) and the amplitude $a$; compute $K_u = 4d/(\pi\sqrt{a^2-h^2})$ by hand.
   Does it match the sketch?
4. Apply a rule: `rule tl` (Tyreus-Luyben), then step the setpoint `sp 1.5` / `sp 3.5`.
   Repeat with `rule zn`. Which would you choose, and why?
5. Optional: `tune 2.0 0.02` and compare $K_u$ with the 5 mV run.

## Part C · Windup, on purpose (12 min)

1. Start the logger (no time limit) and type `kp 2`, `ki 1`, `kd 0`, `lim 0 3` (output
   limited to 3 V), `aw none`, `pid`, `sp 2`; wait about 15 s until steady.
2. Type `sp 4` (unreachable: the output saturates at 3 V), wait 15 s, then `sp 2`.
   Watch `i` and how long `u` stays at 3 V after the setpoint drops.
3. In the same logger session, repeat with `aw clamp` and with `aw backcalc` (type `pid`
   between runs to reset the controller); note the time of each run for the plot.
4. Plot the three runs (`y` and `i`) on one figure.

## Part D · Sample time and derivative noise (if time allows)

- Gains from part B, then `ts 200` (5 Hz instead of 100 Hz): what happens to overshoot?
- `kd` large (e.g. 4) with `filter off`, then `filter 0.7`, then `filter 0.9`: look at `u`.
  Compute the cut-off of each `alpha` at your loop rate.

## Checklist

- [ ] FOPDT fit, relay results (pass 1 and pass 2 if you logged both), two rules' gains
- [ ] windup plot with three anti-windup modes
- [ ] `lim 0 5` and `aw clamp` restored, components back in the box
