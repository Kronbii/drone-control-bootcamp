# Portfolio summary

## One paragraph

*Teach it* is a seven-evening drone-control bootcamp designed for 2nd-4th year engineering
students at Space², the Lebanese student innovation space Rami Kronbi co-organises. It goes
from PID on an RC circuit, through a two-motor seesaw rig that behaves like one axis of a
quadrotor, to quadrotor control and rotor-fault tolerance in simulation, and it turns two
of his own projects into curriculum: the easyPID Arduino library (every lab sketch uses it;
the notebooks run a Python port that is tested against the C++ library) and the rotorfault
simulation study (the research simulator of the fault-tolerance session). Every session has
notes with equations, an auto-checked notebook, a lab, a quiz and slides; there is an
instructor guide, a rig design with safety rules and printable parts, a cohort plan with
costs, a rubric and certificate criteria, and a CI pipeline that executes every notebook
and compiles every sketch. It has not been taught yet: it is ready for a first cohort, with
a pilot plan that says what that cohort must measure.

## Concrete facts

- **7 sessions** of about 3 hours (plus an optional build evening and an open-lab evening),
  each with notes, notebook, lab, quiz and a Marp slide deck (7 PDFs).
- **7 notebooks with 41 auto-checked exercises**, built from one source into student and
  solution versions; CI executes all 14 and checks that student versions fail only at the
  auto-checks.
- **6 labs plus a capstone test day; 8 Arduino sketches** using easyPID 1.1.0, compiled
  warning-free for Uno and Nano (16 builds) in CI.
- **USD 96-168 per team** for a bench kit and a rig (USD 700-1,238 for six teams plus shared
  items), planning ranges before shipping and contingency, generated from a single kit CSV.
- A Python port of easyPID matched the compiled C++ library on **3,274 randomised updates**
  (worst relative difference 4.8×10⁻⁴, float32 vs float64).

## CV lines

- Designed a 7-session drone-control curriculum (PID → attitude estimation → fault-tolerant
  quadrotor control) for university students at Space²: 41 auto-checked Jupyter exercises,
  8 Arduino labs on a custom two-motor rig, instructor guide, rubric and USD 96-168-per-team kit plan.
- Turned my easyPID library into teaching material with a Python port verified against the
  C++ sources (3,274 randomised updates), and found and documented a first-run bias in its
  relay autotuner by testing it against an RC ladder with a known ultimate gain.
- Built the CI for a teaching repository: notebook execution with student/solution checks,
  host-compiled C++ cross-checks, arduino-cli builds of every sketch, and link/consistency tests.

## Suggested portfolio fields

**What.** A ready-to-run, seven-evening bootcamp on drone control -- PID, attitude, fault
tolerance -- with notebooks, labs on a seesaw rig, slides, and a cohort plan.

**Problem.** Control theory is usually taught on paper, and students at Space² have
printers, electronics and enthusiasm but no structured path from "what is PID" to "how does
a drone stay up, and what happens when a propeller breaks".

**How.** Bench electronics first, then one drone axis on a rig with a ground-truth
potentiometer, then the full quadrotor and faults in simulation; every exercise auto-checked,
every lab predicted by a notebook before it is measured, every capstone scored by the same
code on the rig and in simulation, and safety designed in layers (cage, e-stop, guards,
interlocks, roles).

**Role.** Sole author: curriculum, notebooks, Python package, Arduino sketches, rig design
and parts, slides, instructor and cohort documents, CI. Co-organiser of Space², where it is
designed to run.

**Stages.**

| step | title | detail |
|---|---|---|
| 1 | Bench PID | RC circuit and 3-stage RC ladder with easyPID; windup, filtering and relay autotuning measured against known answers |
| 2 | One drone axis | two-motor seesaw rig with a pot for ground truth: thrust curve from a kitchen scale, interlocks, first closed loop |
| 3 | Attitude | MPU-6050 against the pot; complementary and Kalman filters; rate/angle cascade with two easyPID instances |
| 4 | The full vehicle | minimal 6-DOF quadrotor simulator checked against the rotorfault model; mixing priorities and a four-loop cascade |
| 5 | Faults and capstone | signatures, CUSUM and re-allocation with rotorfault; an acceptance test that runs identically on the rig and in simulation |

**Measurements** (all from simulation or software tests; nothing from a cohort yet).

| label | value | context |
|---|---|---|
| easyPID port vs C++ | ≤ 4.8×10⁻⁴ relative | 3,274 randomised updates, 12 scripts, all anti-windup/filter/direction modes |
| Relay autotune, RC ladder | K_u 42 → 28 (exact 29) | first run from rest vs second run straight after, easyPID PIDTuner, simulated with ADC quantisation |
| Cascade vs single loop | 7.0° → 3.0° | peak deviation after a tap on the rig model, single PID on the pot vs rate/angle cascade on the IMU estimate |
| P-only stability limit | K_p ≈ 0.78 µs/deg | rig model with an 80 ms motor lag: why the lab starts with D |
| Minimal quad vs research model | < 10⁻⁴ | state agreement after 0.3 s of open-loop roll command (yaw differs: missing rotor reaction torque) |

**Limits.** Not taught yet; rig parameters are assumptions; printable parts rendered but not
printed; sketches compiled but not run on hardware; costs are planning ranges; all
performance numbers are simulations.

**Hero image.** `figures/hero_arc.png` -- the arc in three panels (PI on the RC circuit,
cascade on the rig model, rotor loss in rotorfault). Alternatives: `figures/s04_cascade.png`
for a single-panel version, `figures/s06_complete_loss.png` for the fault-tolerance story.
