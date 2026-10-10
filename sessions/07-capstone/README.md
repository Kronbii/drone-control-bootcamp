# Session 7 · Capstone: tune it, test it, report it

*One evening, about 3 hours, plus a two-page team report due one week later. Designed for
2nd-4th year engineering students; not yet taught. Track R is props-on
([safety rules](../../hardware/rig/safety.md)).*

Teams of three choose a track at the end of session 6. Both tracks are scored against a
written specification by the same notebook, and both end in the same kind of report.

## Track R · the rig

Tune a controller for the seesaw rig and pass the **acceptance test** that
[`s07_capstone`](../../arduino/s07_capstone/s07_capstone.ino) runs when it receives `test`
(33 s, fully automatic, logged with `tools/serial_logger.py --send test`):

| time (s) | what happens |
|---|---|
| 0-3 | hold 0° |
| 3 | step to +15° |
| 8 | step to -10° |
| 13 | step to 0° |
| 20 | disturbance: +40 µs on the differential command for 100 ms |
| 25 | fault: motor 2's throttle above idle scaled by 0.75 |
| 29 | step to +10° with the fault still active |
| 33 | end (fault removed) |

**Specification** (`bootcamp.capstone.RIG_SPEC`):

| metric | limit |
|---|---|
| overshoot of the +15° step | ≤ 20 % |
| settling of the +15° step into ±1° | ≤ 2.0 s |
| worst mean \|error\| over the last 1 s of each hold | ≤ 1.0° |
| peak deviation after the disturbance | ≤ 8° |
| back inside ±1° after the disturbance | ≤ 1.5 s |
| mean \|error\| 2-4 s after the +10° step with the fault | ≤ 1.5° |
| RMS of the command while holding 0° (chatter) | ≤ 25 µs |

Run the test **three times** and report the worst run. The same test runs on the rig model
(`bootcamp.capstone.simulate_rig_test`): teams validate their gains in notebook 7 first.
The two TEAM HOOKS in the sketch (feedforward, gain scheduling) are optional extras.

## Track S · simulation

Fly the quadrotor waypoint course of `bootcamp.capstone` (five waypoints, 4 s each) with a
50 % loss of effectiveness on rotor 1 at t = 9 s, using `bootcamp.quad`. Notebook 7 gives a
fault-aware allocator driven by an **oracle** (told the truth 50 ms after the fault). The
project: **replace the oracle with your own detector and estimator** (a residual from the
simulated vehicle, a CUSUM, an $\hat\eta$ estimate), and pass the course specification
(`bootcamp.capstone.QuadSpec`): no crash, distance to the current waypoint ≤ 0.6 m once each
leg has had 2 s, final error ≤ 0.10 m. Teams who finish may attempt a complete loss
using rotorfault's yaw-relaxed controller as a reference.

## The evening

| time | what |
|---|---|
| 0:00 | Stand-up: each team states its plan and its current numbers (1 min each) |
| 0:10 | Studio: tuning on the rigs / in simulation, instructors circulate |
| 1:40 | Break |
| 1:50 | Acceptance tests: three logged runs per Track-R team at the test station; Track-S teams freeze code and run the scoring cell |
| 2:30 | Demos: 3 minutes per team, one plot, one number, one surprise |
| 2:50 | Quiz 7 (final concept check), feedback forms, what happens next |

An **optional open-lab evening** between sessions 6 and 7 is strongly recommended for
Track R (hardware time is the bottleneck); see the [cohort plan](../../cohort/plan.md).

## Deliverables

1. The logs (Track R: three `test` runs; Track S: the scoring output and the code).
2. The two-page report, from [writeup-template.md](writeup-template.md), due one week later.
3. The 3-minute demo.

Assessment: [cohort/plan.md](../../cohort/plan.md#assessment-rubric). Passing the spec is
not required for the certificate; an honest, well-measured report of a controller that
fails one line scores better than an unexplained pass.

## Materials

| | |
|---|---|
| Capstone guide | [notes.md](notes.md) |
| Notebook | [`notebooks/s07_capstone.ipynb`](../../notebooks/s07_capstone.ipynb) (5 auto-checked exercises) |
| Test-day protocol | [lab.md](lab.md) |
| Report template | [writeup-template.md](writeup-template.md) |
| Quiz | [quiz.md](quiz.md) |
| Slides | [slides/07-capstone.md](../../slides/07-capstone.md) |
