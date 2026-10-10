# Session 1 · Feedback and PID from first principles

*One evening, about 3 hours. Designed for 2nd-4th year engineering students; this
session has not been taught yet (see [the pilot plan](../../cohort/pilot-plan.md)).*

## After this session you can

- write the model of a first-order plant and predict its step response;
- predict the steady-state error and closed-loop speed of a P controller before simulating it;
- explain what the I and D terms do, and why D matters for plants with inertia;
- write a discrete PID update that matches the **easyPID** Arduino library number for number;
- run easyPID on an Arduino, first around a simulated plant, then around a real RC circuit, and compare the result with a prediction.

## Before the evening (30 min)

- Install the course software ([docs/setup.md](../../docs/setup.md)) and open
  `notebooks/s01_feedback_pid.ipynb` once to check it runs.
- Install the Arduino IDE (or `arduino-cli`) and easyPID 1.1.0 from the Library Manager.
- Prerequisites: first-order ODEs and complex numbers at the level of a 2nd-year
  engineering course; some Python or C. No control course needed.

## The evening at a glance

| time | what | where |
|---|---|---|
| 0:00 | Why feedback? A thermostat, a cruise control, a drone. | [slides](../../slides/01-feedback-and-pid.md) |
| 0:15 | First-order plant, P control, offset and speed | [notes §1-3](notes.md), notebook §1-2 |
| 0:55 | PI and PID; the discrete easyPID update | notes §4-5, notebook §3 (exercise 1.3) |
| 1:25 | Break | |
| 1:35 | Second-order plants and the D term | notes §6, notebook §4-5 |
| 2:00 | **Lab 1**: easyPID on a simulated plant, then on an RC circuit | [lab.md](lab.md) |
| 2:45 | Quiz and wrap-up | [quiz.md](quiz.md) |

Detailed timings, instructor notes and common mistakes: [instructor guide](../../instructor/guide.md#session-1).

## Materials

| | |
|---|---|
| Lesson notes | [notes.md](notes.md) |
| Notebook | [`notebooks/s01_feedback_pid.ipynb`](../../notebooks/s01_feedback_pid.ipynb) (5 auto-checked exercises) |
| Lab | [lab.md](lab.md), sketches [`s01_sim_plant`](../../arduino/s01_sim_plant/s01_sim_plant.ino), [`s01_rc_plant`](../../arduino/s01_rc_plant/s01_rc_plant.ino) |
| Quiz | [quiz.md](quiz.md) (answers in the instructor folder) |
| Slides | [slides/01-feedback-and-pid.md](../../slides/01-feedback-and-pid.md) |

Per team: an Arduino Uno or Nano and USB cable, a breadboard, one 10 kΩ resistor, one
100 µF electrolytic capacitor (≥ 10 V), jumper wires. See the [kit list](../../cohort/kit-and-costs.md).
