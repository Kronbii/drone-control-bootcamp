# Session 2 · Tuning and real-world PID

*One evening, about 3 hours. Designed for 2nd-4th year engineering students; not yet taught.*

## After this session you can

- discretise a first-order plant exactly and explain the phase cost of a slow sample rate;
- recognise integrator windup in a log and choose an easyPID anti-windup mode;
- choose easyPID's derivative-filter coefficient from a cut-off frequency;
- measure rise time, overshoot and settling time from a log, with the course definitions;
- identify a plant from an open-loop step (FOPDT two-point fit) and tune it with the
  reaction-curve rule, with easyPID's relay autotuner (`PIDTuner`), and by hand;
- explain why a tool should be tested on a case with a known answer (the autotuner on the RC ladder).

## Before the evening (20 min)

Finish notebook 1. Build a 3-stage RC ladder on your breadboard at home if you have the
parts (three 10 kΩ, three 100 µF); the lab starts with it.

## The evening at a glance

| time | what | where |
|---|---|---|
| 0:00 | Quiz 1 review; what goes wrong on real hardware | [slides](../../slides/02-tuning-and-real-world-pid.md) |
| 0:10 | Sampling and discretisation | [notes §1](notes.md), notebook §1 |
| 0:30 | Saturation, windup, easyPID anti-windup modes | notes §2, notebook §2 |
| 0:55 | Derivative noise, EMA filter, setpoint kick | notes §3-4, notebook §3-4 |
| 1:20 | Break | |
| 1:30 | Measuring responses; identification; tuning rules; relay autotune | notes §5-7, notebook §5-7 |
| 2:00 | **Lab 2**: step test, autotune, windup on the 3-stage RC ladder | [lab.md](lab.md) |
| 2:45 | Quiz 2 | [quiz.md](quiz.md) |

## Materials

| | |
|---|---|
| Lesson notes | [notes.md](notes.md) |
| Notebook | [`notebooks/s02_real_world_pid.ipynb`](../../notebooks/s02_real_world_pid.ipynb) (8 auto-checked exercises) |
| Lab | [lab.md](lab.md), sketch [`s02_rc_tuning`](../../arduino/s02_rc_tuning/s02_rc_tuning.ino) |
| Quiz | [quiz.md](quiz.md) |
| Slides | [slides/02-tuning-and-real-world-pid.md](../../slides/02-tuning-and-real-world-pid.md) |

Per team: the lab-1 kit plus two more 10 kΩ resistors and two more 100 µF capacitors.
