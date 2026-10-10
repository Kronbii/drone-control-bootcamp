# Safety: rules for the rig

A 10-inch propeller at a few thousand rpm cuts skin, breaks fingernails and throws
fragments when it hits something. The rig is designed so that the first two layers of
protection are physical and do not depend on anyone's code. **These rules are part of
the course and part of the assessment.** The instructor may stop any team at any time.

## The layers

1. **Distance**: props-on work happens with the rig inside the closed mesh cage, the cage
   on a clamped base, and nobody's hands inside it. Nobody stands in the plane of the
   propellers.
2. **Energy cut-off**: the latching e-stop in the 12 V line, outside the cage, within reach
   of the safety officer. Pressed = no motor power, whatever the software does.
3. **Guards**: printed ring guards on both props stop casual contact and catch a thrown
   blade sideways.
4. **Software interlocks** (`arduino/common/rig_safety.h`): arming only on an OFF→ON switch
   transition, plausible pot, beam within ±20°, ≥ 3 s after boot; disarm on switch off, pot
   wire loose, beam at the stops for 300 ms, IMU failure (cascaded sketches) or the `x`
   command; throttle capped at 1600 µs; 2 s ramp-up after arming.
5. **People**: eye protection, roles, and the checklist below.

## Rules

1. **Props off** for: wiring, ESC calibration, motor-direction checks, uploading untested
   code, and any change to the throttle limits. Props go on only after the instructor has
   seen the props-off checks pass.
2. **Safety glasses on** for everyone within 3 m whenever props are fitted and the supply
   is plugged in.
3. **Two roles at every props-on test**: the *operator* at the laptop types commands; the
   *safety officer* keeps one hand next to the e-stop and watches the rig, not the screen.
   Swap roles between tests. Nobody else at the bench.
4. **Cage closed** before the supply is switched on; supply off and e-stop pressed before
   the cage is opened. Wait for the props to stop completely.
5. **Never raise `US_CAP` (1600 µs)** or remove an interlock. Changes to `rig_safety.h` need
   the instructor's sign-off.
6. **Check the props** before each session: no cracks, no nicks, prop nuts tight, correct
   direction (each pushes its end up).
7. **Hair tied back, no loose sleeves, scarves or lanyards** near the cage.
8. **Something unexpected = e-stop first**, questions afterwards: unusual noise, smell,
   vibration, smoke, a motor that does not stop on `x`, an IMU or pot reading that jumps.
9. **No LiPo batteries** in the lab sessions.
10. The supply's mains side stays closed; nobody opens a mains device.

## Pre-power checklist (read aloud, operator and safety officer)

- [ ] Rig base clamped; cage frame intact and closed
- [ ] Props: undamaged, tight, correct direction; guards fitted and bolted
- [ ] Wiring: no bare conductors, ESC BEC wires taped, fuse fitted
- [ ] E-stop tested: pressed → ESCs silent; released → start-up beeps
- [ ] Arm switch OFF; sketch uploaded and Serial Monitor/logger open
- [ ] Glasses on; roles assigned; bystanders behind the line
- [ ] Gains are the "start" values of the lab sheet or values that passed in simulation

## After a session

E-stop pressed, supply unplugged, props removed and stored, sketch with the motors
disabled left on the board (`s03_esc_bench` boots with the motors stopped).

## Incidents

Stop the session, make the area safe, give first aid if needed, and write a short note
for the instructor log (what happened, what layer failed, what changes). A near miss
gets the same note. The first-aid kit (with dressings for cuts) and a CO2/ABC
extinguisher are in the room for every rig session.
