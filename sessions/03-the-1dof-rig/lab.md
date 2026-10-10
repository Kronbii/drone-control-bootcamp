# Lab 3 · Commission the rig, measure it, close the loop

*Teams of 3: operator (laptop), safety officer (e-stop), recorder. Rotate roles between
parts. **Read [safety.md](../../hardware/rig/safety.md) first.** The instructor signs off
part A before any team fits propellers.*

**You need:** your built rig in its cage, supply + e-stop board, Arduino with USB, kitchen
scale, phone stopwatch, safety glasses for everyone, `data/thrust_template.csv`.

**You hand in:** the filled thrust sheet, the free-swing period, the fitted $c_1$, $c_2$, $h$,
and one logged step response (`seq`) with the gains you ended with, plotted next to the
notebook-3 prediction for those gains.

## Part A · Props-off commissioning (30 min) -- sign-off required

1. **Visual check** against [wiring.md](../../hardware/rig/wiring.md): fuse, e-stop, ESC
   BEC wires taped, signals on D9/D10, signal grounds on GND, pot on A0, arm switch on D2.
2. Upload `s03_esc_bench`, open the Serial Monitor (115200), arm switch **OFF**.
3. Plug in the supply, release the e-stop: the ESCs play their start-up tones, then stay
   quiet. Press the e-stop: everything dead. Release again. *(This is the e-stop test of
   the checklist; repeat it at every session.)*
4. **Pot.** Send `zero` with the beam level; note `POT_CENTER`. Tilt the motor-1 end up by
   hand: `pot_deg` must increase. If it decreases, set `potSign = -1.0f` in the rig sketches.
5. **Arming interlocks.** Try to arm with the switch already ON at power-up (must not arm:
   it needs an OFF→ON transition). Arm properly (switch OFF→ON): `# ARMED`, LED on. Then,
   one at a time, check each disarm: switch OFF; pull the pot wiper wire (the 1 MΩ
   pull-down makes it read 0: `pot reading implausible`); send `x`. Each must print `# DISARMED: ...` and stop the motors.
6. **Direction.** Armed, send `m1 1150`: motor 1 spins slowly. Mark its direction. Same with
   `m2 1150`. Each prop, once fitted, must push its end **up** (CW and CCW props as needed;
   swap two motor wires to reverse a motor). `x`.
7. Optional, only if your ESCs need it (many SimonK/generic ESCs do, BLHeli usually not):
   ESC throttle-range calibration with `cal props-off`, following the printed instructions.

**Instructor sign-off:** ______ (e-stop works, interlocks work, directions marked)

## Part B · Thrust curve with a kitchen scale (20 min) -- props on, cage closed

1. Fit props and guards (prop nuts tight). Rest the motor-2 end of the beam on the kitchen
   scale with a spacer so the beam is level; tare the scale. Close the cage.
2. Checklist read aloud. Arm. Send `sweep 1`: motor 1 steps 1100 → 1600 µs, 5 s per step.
3. At every `# STEP` line, wait ~3 s and write the scale reading into
   `data/thrust_template.csv` (save your copy as `logs/thrust_team<N>.csv`).
4. `x`, e-stop, then open the cage. Repeat for motor 2 if time allows (scale under the
   motor-1 end, `sweep 2`): are the two motors the same?
5. In notebook 3, replace the example data with yours and run exercises 3.1-3.2: you now
   have $c_1$, $c_2$ and the torque gain of *your* rig.

## Part C · Free swing (10 min) -- supply off

1. Supply off, e-stop pressed. Lift one end by about 5° and let go. Time 10 full swings
   (or log `pot_deg` with the logger and count zero crossings).
2. Weigh the moving parts (or use the build log). Notebook exercise 3.3 gives $h$.
3. Change the CG adjusters (two nuts more or less on each bolt) and time it again: does $h$
   move the way you expect?

## Part D · First closed loop (35 min) -- props on, cage closed

1. Upload `s03_rig_angle_pid`. Put your `POT_CENTER` (from part A) into `potCenter`, or send
   `zero` after every reset.
2. Checklist. Arm (switch OFF→ON). The base throttle ramps up over 2 s; the loop starts
   with `kp 0.5 ki 0 kd 0.3`.
3. Start the logger (`--seconds 25 logs/lab3_run1.csv`) and send `seq` (0, +10, -10, 0°, 4 s
   each). Disarm.
4. Plot the log in notebook 3 next to the simulated response for the same gains
   (`rig.simulate_rig` with your measured parameters). Where do they differ?
5. Walk the gains up, one change per run, each about 25 %, logging every run: first `kd`,
   then `kp`, then `ki` to remove the offset. Stop when you meet the notebook-3
   specification (overshoot < 15 %, settling into ±1° < 2 s) or when the motors start to
   sound rough (noise through the D term: try `filter 0.7`).
6. Write down your final gains; you will need them in session 4.

Never: P-only with `kp` above 0.5 on the rig, `base` above 1500, or opening the cage while
the supply is on.

## Checklist before you leave

- [ ] thrust sheet, swing period, $c_1$, $c_2$, $h$, final gains recorded in the team log
- [ ] e-stop pressed, supply unplugged, props removed and stored
- [ ] sketch with motors stopped (`s03_esc_bench`) left on the board
