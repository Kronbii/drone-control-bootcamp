# Session 3 · The seesaw rig: one axis of a drone

*One evening, about 3 hours. Designed for 2nd-4th year engineering students; not yet taught.
The first session with spinning propellers: [the safety rules](../../hardware/rig/safety.md)
are part of the session.*

## After this session you can

- explain how differential thrust produces a torque, and write the rig's equation of motion;
- measure a motor's thrust curve with a kitchen scale and turn it into a torque gain;
- find the rig's centre-of-gravity offset from its free-swing period;
- show with closed-loop poles why proportional control alone fails on this rig;
- implement the mixer and the arming interlocks, and say which safety layer each one is;
- close an angle loop with easyPID on real hardware, starting from gains validated in simulation.

## Before the evening (30 min)

- Read [hardware/rig/safety.md](../../hardware/rig/safety.md) and the [rig overview](../../hardware/rig/README.md).
- Do notebook 3 sections 1-3 (the thrust fit can use the example data).
- The rigs are built beforehand ([build guide](../../hardware/rig/build.md)); arrive
  ready to check and wire, not to cut wood.

## The evening at a glance

| time | what | where |
|---|---|---|
| 0:00 | Safety briefing and roles; e-stop demonstration | [safety.md](../../hardware/rig/safety.md) |
| 0:15 | The rig as a drone axis: torque, mixer, model | [notes §1-3](notes.md), [slides](../../slides/03-the-1dof-rig.md) |
| 0:40 | **Lab A**: props-off commissioning (ESC, direction, interlocks) | [lab.md](lab.md) |
| 1:10 | Break | |
| 1:20 | **Lab B-C**: thrust curve and free swing → model parameters | lab.md, notebook 3 §1-2 |
| 1:50 | Why P fails; gains in simulation | notes §4-5, notebook 3 §3-6 |
| 2:10 | **Lab D**: first closed loop, gains walked up from the safe start | lab.md |
| 2:45 | Quiz 3 | [quiz.md](quiz.md) |

## Materials

| | |
|---|---|
| Lesson notes | [notes.md](notes.md) |
| Notebook | [`notebooks/s03_rig.ipynb`](../../notebooks/s03_rig.ipynb) (6 auto-checked exercises) |
| Lab | [lab.md](lab.md); sketches [`s03_esc_bench`](../../arduino/s03_esc_bench/s03_esc_bench.ino), [`s03_rig_angle_pid`](../../arduino/s03_rig_angle_pid/s03_rig_angle_pid.ino) |
| Hardware | [hardware/rig/](../../hardware/rig/) (BOM, build, wiring, safety, printable parts) |
| Quiz | [quiz.md](quiz.md) |
| Slides | [slides/03-the-1dof-rig.md](../../slides/03-the-1dof-rig.md) |
