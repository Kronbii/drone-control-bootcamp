# Capstone guide

How to get from "it sort of works" to numbers you can defend, in one week of team time.

## 1. Work like this

1. **Read the spec and the test.** Every line of `RIG_SPEC` (or `QuadSpec`) is computed by
   `bootcamp.capstone.evaluate_rig_test` / `evaluate_quad_course`; read the ten lines of each.
2. **Model first.** Put your measured parameters (session 3: thrust curve, mass, swing
   period) into `bootcamp.rig.RigParams` and run `simulate_rig_test` with your controller in
   notebook 7, section 3. Find gains that pass on two seeds with margin.
3. **Hardware second, gently.** Start the rig at about half the simulated gains, run `seq`
   and `kick`, raise towards the simulated values in 25 % steps, logging every run.
4. **Test the way you will be scored.** Run `test` with the logger; analyse with notebook 7.
   Change one thing at a time; keep a table of runs (gains → each spec line).
5. **Explain the gap.** Where the hardware differs from the model, find out why: a weak
   motor, more friction, vibration in the gyro, a CG offset different from your estimate.
   The gap is part of the result.

## 2. What usually limits each spec line (rig)

| line | what usually limits it | what helps |
|---|---|---|
| step overshoot | outer-loop gain too high for the inner loop's speed; integral windup on large steps | lower `akp`, faster inner loop, outer `aki` small or zero |
| step settling | weak inner loop; friction; motor mismatch | raise `rkp`; `rki` for friction; check both thrust curves |
| hold error | CG offset and motor mismatch with no integral | small `aki` (outer) or `rki` (inner) |
| disturbance peak | inner-loop stiffness | raise `rkp` (watch chatter) |
| disturbance recovery | outer loop / integral speed | `akp`, `rki` |
| fault error | the steady torque the weak motor removes needs integral action | `rki` or `aki`; a feedforward hook if you can estimate it |
| chatter (u RMS) | gyro vibration through `rkp`/`rkd` | EMA filter on the rate D term, lower `rkd`, better vibration isolation of the IMU |

## 3. Track S: from oracle to detector

The oracle in notebook 7 sets `eta_hat` 50 ms after the fault. A minimal replacement:

1. **Residual**: the vehicle's angular acceleration predicted from the commanded rotor
   thrusts, $J^{-1}(B_\tau f - \omega\times J\omega)$, minus the measured one (finite difference
   of `quad.w`); add the vertical force balance for a fourth row.
2. **Isolation**: project on the signatures $d_j = -B_{:,j}f_j$ (session 6).
3. **Detection**: a CUSUM per rotor on the projected statistic, threshold set from
   fault-free runs of the course (not from the faulty one).
4. **Estimation**: after the alarm, a least-squares or recursive estimate of $\eta_j$ from the
   same regression; feed it to `eta_hat`.
5. **Report** detection delay, isolation correctness and the course metrics, with and
   without your FDI, on several seeds (add noise with `gyro_noise` so the test is not trivial).

## 4. The report

Two pages, [writeup-template.md](writeup-template.md). It is assessed on the rubric in the
cohort plan: correctness of the numbers, quality of the evidence (plots that show the
claim), honesty about failures and model gaps, and clarity. Numbers without a log behind
them score zero.

## 5. Common capstone mistakes

- Tuning by watching the Serial Plotter instead of logging and computing the spec.
- Changing three gains between runs.
- Reporting the best of three runs instead of the worst.
- A controller that passes in simulation but was never re-checked after a hardware change.
- Turning `US_CAP` or the interlocks up "just for the test". Not allowed.
