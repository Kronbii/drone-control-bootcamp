# Capstone write-up · Team ___ · Track R / S

*Two pages maximum including figures. Every number must come from a log or a notebook
run that you include. Delete the italic guidance.*

**Team:** names · **Rig / seed set:** ___ · **Date of the test runs:** ___

## 1. Result

| spec line | limit | run 1 | run 2 | run 3 | worst | pass? |
|---|---|---|---|---|---|---|
| step overshoot (%) | ≤ 20 | | | | | |
| step settling (s) | ≤ 2.0 | | | | | |
| hold error (°) | ≤ 1.0 | | | | | |
| disturbance peak (°) | ≤ 8 | | | | | |
| disturbance recovery (s) | ≤ 1.5 | | | | | |
| fault error (°) | ≤ 1.5 | | | | | |
| command RMS (µs) | ≤ 25 | | | | | |

*Track S: replace with the course metrics (crash, max error after the fault, final error),
detection delay and isolation, with and without your FDI, over seeds 0-4.*

*For every FAIL: one sentence on why, from your data.*

## 2. Controller

*Structure (single loop / cascade / hooks used), final gains and filter settings, and the
commands you typed. How you found the gains: simulation first? how many hardware runs?*

## 3. Evidence

*Figure 1: the acceptance-test plot of your worst run (notebook 7). Figure 2: one plot you
chose because it shows something the table does not (for example the gyro noise, a
comparison with the simulation, the effect of one gain). Caption each with what to look at.*

## 4. Model versus hardware

*Your simulated result for the same gains (notebook 7, section 3) next to the measured one.
Where do they differ, and what is your best explanation? Which measured parameter (thrust
curve, CG offset, motor lag, friction) would you re-measure first?*

## 5. With one more week

*One change to the controller or the rig, and the measurement that would tell you whether
it worked.*

## Contributions

*One line per team member.*
