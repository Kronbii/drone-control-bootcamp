# Lab 4 · What the IMU really says, and a flight-controller-style cascade

*Teams of 3 with the session-3 roles. Props on for parts A2 and B only, cage closed,
checklist read aloud ([safety.md](../../hardware/rig/safety.md)).*

**You need:** your rig with the GY-521 mounted on the pivot axis (x along the beam towards
motor 1), USB, logger, your session-3 gains.

**You hand in:** (1) one plot of pot / accelerometer / gyro-integrated / complementary angle
with motors off and one with motors on, with the RMS error of each estimate against the
pot; (2) one logged `seq` run with the cascade on the IMU estimate, compared with your best
single-loop run from session 3; (3) your cascade gains.

## Part A · The IMU against the pot (30 min)

1. Upload `s04_imu_check`. **Keep the beam still** during the 2 s gyro calibration after
   reset; note the printed `WHO_AM_I` and gyro bias.
2. **A1, motors off.** Start the logger (`--seconds 30 logs/lab4_off.csv`) and move the beam
   slowly by hand, then quickly, then hold it at about +15° and -15°.
   - Do `pot`, `acc`, `gyro_int` and `comp` agree in sign? (If not: fix the mounting or
     `BEAM_GYRO_SIGN`, see [build.md](../../hardware/rig/build.md) §4.)
   - Which estimate drifts? Which is noisy during fast moves?
3. **A2, motors on.** `s04_imu_check` has no controller, so rest the beam level on a block,
   close the cage, read the checklist, arm and send `both 1300` (both motors at a constant
   1300 µs: equal thrust, no net torque). Log 20 s (`logs/lab4_on.csv`), then `x`. Compare
   the accelerometer-angle noise with A1: that is propeller vibration.
4. In notebook 4 (section 3), load both logs with `bootcamp.logs.read_log` and compute the
   RMS error of `acc`, `gyro_int` and `comp` against `pot`. Try `tau 0.2`, `tau 0.5`,
   `tau 2` (re-log A2 or re-run the filter offline on your logged `acc` and `rate` with
   `complementary()` from exercise 4.4). Which $\tau$ is best on *your* rig?

## Part B · Cascaded control (35 min)

1. Upload `s04_rig_cascaded`; set `potCenter` (or send `zero`). It starts with
   `src pot`, `akp 4 aki 0 | rkp 0.8 rki 0.6 rkd 0.01`, rate limit 150 deg/s.
2. **Inner loop first.** Checklist, arm. With `akp 1` (soft outer loop), send `sp 5` and
   `sp -5` and watch `rate` follow `rate_sp` in the log. Raise `rkp` in steps of ~25 %
   (0.8 → 1.0 → 1.25 → ...) until the rate tracks crisply without buzzing; add `rki` until
   the rate error settles; keep `rkd` small. The notebook-4 simulation ended near
   `rkp 3 rki 1 rkd 0.05` on the *model* -- your rig decides.
3. **Outer loop.** Raise `akp` (2 → 4 → 6) and run `seq`, logging each run.
4. **Disturbance test.** With the beam holding 0°, send `kick`: the sketch adds 40 µs to the
   differential command for 100 ms -- a repeatable push, the same as in the capstone test.
   Log three kicks and note the peak deviation. Compare with the session-3 single loop
   (`s03_rig_angle_pid` has the same `kick` command). Never push the beam by hand while
   the props turn.
5. **Drone mode.** `src imu`: the outer loop now uses the complementary filter, not the pot.
   Repeat one `seq`. The pot still logs the truth: how much worse is the IMU-based loop?
6. Record your final gains.

## Checklist before you leave

- [ ] logs and plots for parts A and B, final gains in the team log
- [ ] e-stop pressed, supply unplugged, props removed
