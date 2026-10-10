# Instructor guide

For the people running the bootcamp at Space². It assumes the cohort described in the
[cohort plan](../cohort/plan.md): about 18 students in 6 teams of 3, one evening a week,
3 hours per evening, two instructors (one of them comfortable with the rig hardware) and,
for the props-on sessions 3, 4, 6 and 7, one extra helper.

**Status: designed, not yet taught.** Timings are estimates; the [pilot plan](../cohort/pilot-plan.md)
says how to check them. Please log what actually happened after every evening.

## Principles

1. **Predict, then measure.** Every lab starts from a number the notebook predicted; the
   discussion is about the gap.
2. **Notebooks never block.** The auto-checks tell students where they are; the notebook
   continues with reference code if they are stuck. Do not let an evening stall on one TODO:
   point to the check's hint, then move on.
3. **Safety is taught, not just enforced.** The interlocks are an exercise (3.5), the rules
   are on the quiz, and the props-on sign-off is part of lab 3.
4. **One change at a time, and log it.** Model this yourself when you demonstrate.

## Before the cohort

See the timeline in the [cohort plan](../cohort/plan.md#timeline). In short: order motors,
ESCs and props 6+ weeks ahead; print parts and cut wood 3 weeks ahead; build and commission
every rig yourself (props-off, then props-on) at least 1 week before session 3; run every
notebook and `make test` on the room's network and on a student-like laptop.

## Room setup

- Sessions 1, 2, 5: tables in team islands, a projector, mains for laptops. Each team box:
  Arduino, USB cable, breadboard, RC parts.
- Sessions 3, 4, 6 (part B), 7: rigs on a separate row of tables along a wall, clamped,
  cages on, a taped **line 1.5 m in front** that nobody crosses while props turn, glasses at
  the door, first-aid kit and extinguisher visible. Power boards (supply, fuse, e-stop)
  outside the cages. A second table row for laptops/simulation work.

---

## Session 1

| time | activity | instructor notes |
|---|---|---|
| 0:00-0:15 | Welcome, how the bootcamp works, why feedback (thermostat, cruise control, drone) | Show the hero figure from the README: where we will be in 6 weeks. Check everyone opened notebook 1 at home; pair anyone whose install failed with a team-mate. |
| 0:15-0:55 | First-order plant, P control, offset formula | Derive $r/(1+KK_p)$ on the board; students do exercises 1.1-1.2. Ask for a prediction before every plot. |
| 0:55-1:25 | PI/PID, the easyPID update table | Exercise 1.3 is the heart of the session: the student PID must match easyPID to 1e-9. Common failure: dt in ms used as seconds. |
| 1:25-1:35 | Break | |
| 1:35-2:00 | Second-order plants, PD design | Exercises 1.4-1.5. Demo: show that $K_d$ on a first-order plant does nothing useful. |
| 2:00-2:45 | Lab 1 | Part A for everyone (no wiring). Part B: walk round and check capacitor polarity *before* USB is plugged in. Teams that finish do part C. |
| 2:45-3:00 | Quiz 1 (10 min), questions | Collect quizzes or self-mark with the key. |

**Common mistakes.** `pid.update(sp, y, 0.01)` for 10 ms; reading the final value off the
Serial Monitor while it still moves; capacitor reversed (it gets warm: unplug at once);
two programs trying to open the serial port (close the Serial Plotter before logging);
Linux laptops without permission on `/dev/ttyUSB0` (user must be in the `dialout` group,
log out and in); CH340 driver missing on Windows/macOS for Nano clones.

**Misconception to address.** "The integral makes it faster." It makes the steady state
exact; it usually makes the transient slower or more oscillatory.

## Session 2

| time | activity | instructor notes |
|---|---|---|
| 0:00-0:10 | Quiz 1 review | Q7 (saturation) leads straight into tonight. |
| 0:10-0:30 | Sampling, ZOH, phase cost | Exercises 2.1-2.2. The `s02_sample_time` figure is the hook. |
| 0:30-0:55 | Windup and anti-windup | Exercise 2.3. Show the windup figure; ask what the I-term trace means physically. |
| 0:55-1:20 | Derivative noise, EMA, setpoint kick | Exercises 2.4-2.5. Point out that the same `alpha` is a different filter at a different loop rate. |
| 1:20-1:30 | Break | |
| 1:30-2:00 | Metrics, FOPDT, relay autotune | Exercises 2.6-2.8, then the relay study: the first-run bias is a good "test your tools" story. |
| 2:00-2:45 | Lab 2 | The 3-stage ladder is slow (time constants of seconds): start the open-loop step (part A) early and do the analysis while it runs. Part B takes ~1 minute of relay time for two passes. |
| 2:45-3:00 | Quiz 2 | |

**Common mistakes.** Ladder capacitors charged from a previous run (wait or short them
through a resistor); measuring on A0 instead of A2 (`tap 2`); relay noise band below the
ADC noise (relay chatters, the tuner reports nonsense: use 5 mV or more); forgetting to
restore `lim 0 5` and `aw clamp` after the windup experiment.

## Session 3

The first props-on evening. **Both instructors and the helper must be present.**

| time | activity | instructor notes |
|---|---|---|
| 0:00-0:15 | Safety briefing | Walk through safety.md, assign roles per team, demonstrate the e-stop on a running rig (props on, low throttle) from behind the line. |
| 0:15-0:40 | The rig as a drone axis; model; mixer | Notes §1-3. |
| 0:40-1:10 | Lab A: props-off commissioning | **Sign off each team personally**: e-stop, interlocks (try arming with the switch left on at power-up), motor directions marked. No sign-off, no props. |
| 1:10-1:20 | Break | |
| 1:20-1:50 | Lab B-C: thrust curve, free swing | One instructor at the rigs, one with the notebook. Teams without sign-off do the free swing (supply off) and the notebook. |
| 1:50-2:10 | Why P fails; designing gains in simulation | Notebook 3 §3-6. The pole plot (`s03_p_only_instability`) is the key image. |
| 2:10-2:45 | Lab D: first closed loop | Start gains only (`kp 0.5 ki 0 kd 0.3`). Watch for teams typing the simulated gains first. |
| 2:45-3:00 | Quiz 3, pack down | Props off, supplies unplugged before anyone leaves. |

**Common mistakes.** Props on backwards (the beam is pushed down; at low throttle it is
obvious from the airflow); `potSign` wrong, so the loop pushes the wrong way (the trip at
the stops catches it -- this is a good moment to show that the interlock worked); the ESC
BEC red wire connected to the Arduino 5 V; loose prop nuts (check every session); scale
not tared with the beam resting on it.

**Hardware troubleshooting.** ESC beeps continuously: no valid signal (check D9/D10 and the
signal ground) or throttle not at minimum at power-up. Motor stutters at low throttle:
normal for some ESCs below ~1100 µs (that is why `US_IDLE` is 1100). Motor does not start:
ESC needs throttle-range calibration (`cal props-off`, props off). Pot reading jumps: worn
pot or loose coupler set screw.

## Session 4

| time | activity | instructor notes |
|---|---|---|
| 0:00-0:10 | Quiz 3 review | |
| 0:10-0:40 | Rotations, Euler angles, quaternions | Exercises 4.1-4.2. Physical demo of gimbal lock with a phone or a toy plane. |
| 0:40-1:15 | IMU, complementary filter, Kalman preview | Exercises 4.3-4.5. |
| 1:15-1:25 | Break | |
| 1:25-1:55 | Lab A: IMU against the pot | Check IMU signs by hand-tilting *before* any team arms. |
| 1:55-2:10 | Cascaded loops, tuning inside out | Notebook 4 §6-7. |
| 2:10-2:45 | Lab B: cascade on the rig | Inner loop first with a soft outer loop. Use `kick`, never hands, for disturbances. |
| 2:45-3:00 | Quiz 4, pack down | |

**Common mistakes.** Beam moved during the boot-time gyro calibration (bias of several
deg/s, the estimate drifts: reset with the beam still); I²C dropouts with motors running
(route wires away from the motor leads; the cascaded sketches disarm on an IMU read
failure, which is the intended behaviour); IMU board mounted rotated (fix the mount
rather than the code if possible); tuning `akp` before the inner loop is stable.

## Session 5

| time | activity | instructor notes |
|---|---|---|
| 0:00-0:10 | Quiz 4 review | |
| 0:10-0:40 | Dynamics and allocation | Exercises 5.1-5.2; derive one column of $B$ on the board with the right-hand rule. |
| 0:40-1:10 | Saturation priorities, acceleration to tilt | Exercises 5.3-5.4. |
| 1:10-1:20 | Break | |
| 1:20-1:50 | The cascade; position tuning | Exercise 5.5 (the velocity-integrator insight). |
| 1:50-2:35 | Lab 5: failure catalogue | Teams pick at least four experiments. Simulations take seconds each; encourage sweeps. |
| 2:35-2:45 | Minimal vs research model | Notebook §8 and the reflection. |
| 2:45-3:00 | Quiz 5 | Announce the capstone tracks; teams start thinking. |

**Common mistakes.** Reusing a `CascadedController` between runs (integrator state carries
over); sign confusion in the tilt map (positive pitch accelerates *forward* in this z-up
convention); expecting the minimal model to include noise and delay.

## Session 6

| time | activity | instructor notes |
|---|---|---|
| 0:00-0:10 | Quiz 5 review | |
| 0:10-0:30 | LOE model, hover limit, passive behaviour | Exercise 6.1 and the passive-cascade figure. |
| 0:30-1:10 | Residuals, isolation, CUSUM | Exercises 6.2-6.4; spend time on the delay/false-alarm trade-off plot. |
| 1:10-1:20 | Break | |
| 1:20-1:55 | Re-allocation; rotorfault; complete loss | Exercises 6.5-6.6. Show the complete-loss figure and, if possible, the rotorfault README's 3D figure. |
| 1:55-2:35 | Lab 6 | Part A for all; part B for teams with a commissioned rig and a helper present. |
| 2:35-2:45 | Reading the limits | Read section 6 of the rotorfault README together. |
| 2:45-3:00 | Quiz 6; capstone teams and tracks fixed | |

**Common mistakes.** Mixing up LOE and $\eta$; judging a pipeline from one seed; a CUSUM
threshold tuned on the faulty run; treating rotorfault's numbers as flight results.

**About the source material.** rotorfault is the course author's own simulation study.
Present it as what it is: a reproducible simulation with assumed parameters, a stated
list of limitations and a robustness sweep that shows where it fails. That framing is the
lesson.

## Session 7

See the [capstone README](../sessions/07-capstone/README.md) and the
[test-day protocol](../sessions/07-capstone/lab.md). Instructor jobs: keep the test
stations moving (6 minutes per team), check `rig_safety.h` is unchanged before each team's
test (`python tools/sync_sketch_headers.py --check` on their copy), and keep demos to 3
minutes. Collect the feedback forms before people leave.

---

## Running the labs safely

- **Ratio.** At most two rigs running props-on per adult supervisor who is watching (not
  helping with code). With 6 rigs that means three supervisors, or running rigs in two shifts.
- **Sign-off.** No team fits props before its lab-3 part A sign-off; a team that changes
  wiring later is re-checked.
- **Line and cage.** Nobody crosses the line while a supply is on; cages closed before power.
- **Stop words.** Anyone may shout "stop": the safety officer presses the e-stop, everyone
  steps back. Discuss afterwards.
- **Incidents and near misses** are written down the same evening (what happened, which
  layer failed, what changes), and the change is made before the next session.
- **End of evening.** E-stops pressed, supplies unplugged, props removed and counted.

## Running with shared rigs

With three rigs for six teams (the lean kit option), pair teams on a rig. In sessions 3, 4
and 6 split the lab block: team A on the rig while team B works on the notebook part of
the lab (thrust-curve fit with A's data, IMU log analysis, rotorfault experiments), swap at
half time. Session 7: Track S suits half of the teams anyway; the remaining Track R teams
share rigs and book them in the open-lab evening.

## Notebooks, solutions and auto-checks

- Students work in `notebooks/` (TODOs); solutions are in `solutions/notebooks/`. In a
  public repository the solutions are visible: the exercises are for learning, not graded.
  If you want them hidden, give students a copy of the repository without `solutions/`.
- The checks call `bootcamp.*` reference implementations; a student could import them
  instead of writing code. That is fine for a bootcamp: grade the capstone, not the checks.
- `make test` executes every notebook, student and solution version, and checks that the
  student versions fail only at the auto-check cells. Run it after editing a notebook source
  in `notebooks/src/` (then `make notebooks` to rebuild).

## Assessment workflow

Quizzes are formative (self-marked with [the key](quiz-keys.md)), except quiz 7, which
counts towards the certificate. The capstone is marked with the rubric in the cohort plan by
two instructors independently; disagreements of more than one band are discussed.

## Troubleshooting setup

| symptom | fix |
|---|---|
| `ModuleNotFoundError: bootcamp` in Jupyter | kernel is not the course venv: `python -m ipykernel install --user --name bootcamp` from the venv and select it |
| `rotorfault` will not install (no network / repo not public yet) | instructor brings a wheel on USB: `pip install rotorfault-*.whl`; build it with `pip wheel <rotorfault checkout> -w dist` |
| Arduino: `easyPID.h: No such file` | Library Manager → easyPID 1.1.0, or `arduino-cli lib install easyPID@1.1.0` |
| Arduino: port not found (Linux) | `sudo usermod -aG dialout $USER` on the student's own laptop, log out/in |
| Nano clone will not upload | Tools → Processor → "ATmega328P (Old Bootloader)" (`arduino:avr:nano:cpu=atmega328old`) |
| Logger: `could not open port` | close the Serial Monitor/Plotter; one program per port |
