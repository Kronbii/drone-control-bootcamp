# Session 4 · Rotations, attitude estimation and cascaded loops

*One evening, about 3 hours. Designed for 2nd-4th year engineering students; not yet taught.
Props-on lab: [safety rules](../../hardware/rig/safety.md) apply.*

## After this session you can

- build rotation matrices from Z-Y-X Euler angles, use them to change frames, and spot gimbal lock;
- compose rotations with quaternions;
- explain what a gyroscope and an accelerometer measure, and when each one lies;
- write and tune a complementary filter, and describe what a Kalman filter adds;
- run a cascaded rate/angle controller (two easyPID instances) on the rig, on the IMU
  estimate, and show that it rejects disturbances better than a single loop.

## Before the evening (30 min)

Notebook 4 sections 1-2 (rotations and quaternions). Bring your final session-3 gains.

## The evening at a glance

| time | what | where |
|---|---|---|
| 0:00 | Quiz 3 review; from one axis to three | [slides](../../slides/04-rotations-and-attitude.md) |
| 0:10 | Rotation matrices, Euler angles, gimbal lock, quaternions | [notes §1-2](notes.md), notebook §1-2 |
| 0:40 | IMU sensing; complementary filter; Kalman preview | notes §3-5, notebook §3-5 |
| 1:15 | Break | |
| 1:25 | **Lab A**: pot vs accelerometer vs gyro vs filter, motors off and on | [lab.md](lab.md) |
| 1:55 | Cascaded loops: why and how to tune them | notes §6, notebook §6-7 |
| 2:10 | **Lab B**: cascaded control on the rig | lab.md |
| 2:45 | Quiz 4 | [quiz.md](quiz.md) |

## Materials

| | |
|---|---|
| Lesson notes | [notes.md](notes.md) |
| Notebook | [`notebooks/s04_attitude.ipynb`](../../notebooks/s04_attitude.ipynb) (6 auto-checked exercises) |
| Lab | [lab.md](lab.md); sketches [`s04_imu_check`](../../arduino/s04_imu_check/s04_imu_check.ino), [`s04_rig_cascaded`](../../arduino/s04_rig_cascaded/s04_rig_cascaded.ino) |
| Quiz | [quiz.md](quiz.md) |
| Slides | [slides/04-rotations-and-attitude.md](../../slides/04-rotations-and-attitude.md) |
