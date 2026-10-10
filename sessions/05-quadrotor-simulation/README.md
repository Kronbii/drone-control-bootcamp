# Session 5 · Quadrotor dynamics and control in simulation

*One evening, about 3 hours. Designed for 2nd-4th year engineering students; not yet taught.
Simulation only: no props tonight.*

## After this session you can

- write the 6-DOF rigid-body equations of a quadrotor and the X-configuration allocation matrix;
- compute the hover point and the thrust-to-weight margin;
- explain how a mixer prioritises roll/pitch over thrust over yaw when the motors saturate;
- map a desired acceleration to a tilt command, and describe the position → velocity →
  attitude → rate cascade and its time-scale separation;
- tune the outer loops of the cascade to a specification in simulation;
- judge what a minimal model leaves out by comparing it with a research model (rotorfault).

## Before the evening (20 min)

Read `src/bootcamp/quad.py` once (about 300 lines). Finish notebook 4 section 6.

## The evening at a glance

| time | what | where |
|---|---|---|
| 0:00 | Quiz 4 review; the rig was one axis -- now all six | [slides](../../slides/05-quadrotor-simulation.md) |
| 0:10 | Rigid-body dynamics; rotors; the allocation matrix | [notes §1-2](notes.md), notebook §1-2 |
| 0:40 | Saturation and mixing priorities; acceleration to tilt | notes §3-4, notebook §3-4 |
| 1:10 | Break | |
| 1:20 | The full cascade; tuning the position loop | notes §5, notebook §5-6 |
| 1:50 | **Lab**: the simulation is the lab -- break it on purpose | [lab.md](lab.md) |
| 2:35 | Minimal model vs research model | notebook §8 |
| 2:45 | Quiz 5 | [quiz.md](quiz.md) |

## Materials

| | |
|---|---|
| Lesson notes | [notes.md](notes.md) |
| Notebook | [`notebooks/s05_quad_sim.ipynb`](../../notebooks/s05_quad_sim.ipynb) (5 auto-checked exercises) |
| Simulator | [`src/bootcamp/quad.py`](../../src/bootcamp/quad.py); the study model: `rotorfault` |
| Lab | [lab.md](lab.md) |
| Quiz | [quiz.md](quiz.md) |
| Slides | [slides/05-quadrotor-simulation.md](../../slides/05-quadrotor-simulation.md) |
