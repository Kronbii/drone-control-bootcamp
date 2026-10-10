# Building the rig

About 3 hours per rig for two people with basic tools (drill, saw, files, screwdriver,
soldering iron). Plan a **build evening before session 3** (volunteers, the instructor
team or the cohort itself, see the [cohort plan](../../cohort/plan.md)): session 3 is for
wiring checks, thrust measurements and the first closed loop, not for carpentry.

Tools: drill with 4, 6.5, 8.5 mm bits (and a 22 mm Forstner bit if you do not print the
bearing blocks), hacksaw, file, square, tape measure, screwdrivers, 13 mm spanners (M8),
soldering iron, multimeter, G-clamps.

## 1. Base and uprights

1. Cut the base (400 x 250 mm) and two uprights (320 x 80 mm) from 18 mm plywood.
2. Mark the axle centre on each upright, 280 mm above the base surface and centred across
   the width. Either drill a 22 mm hole 7 mm deep for the 608 bearing (press fit) and an
   8.5 mm through hole, or print two `bearing_block`s and screw them on, centred on the mark.
3. Screw the uprights to the base 60 mm apart (inside faces), square to the base, with the
   holes aligned: push the threaded rod through both before tightening the screws.

## 2. Beam and axle

1. Cut the aluminium tube to 600 mm; deburr. Mark the exact centre and the motor positions
   at 250 mm either side.
2. Drill an 8.5 mm hole through the centre (both walls), square to the tube.
3. Thread an M8 nut and washer onto the rod, slide on the beam, add washer and nut, and
   tighten so the beam is clamped and turns with the rod. Pass the rod through both
   bearings; lock-nuts on the outside faces leave 0.5 mm end play.
4. The beam must swing freely: with no motors it should rock for many seconds.

## 3. Motors, guards and the CG adjusters

1. Fit the motors with `motor_mount`s (or a wooden block and two hose clamps), axes
   vertical, 250 mm from the pivot. Run the motor wires along the beam to the centre
   with cable ties, leaving slack at the pivot.
2. Fit props later (lab 3, part B) -- not now.
3. Bolt the four `prop_guard_segment`s of each guard into a ring; bolt the two spokes to
   the motor mount. Check by hand that a prop (when fitted) cannot touch the ring.
4. Drill two 6.5 mm holes in the bottom face of the beam, 30 mm either side of the axle,
   and hang an M6 x 60 mm bolt (head inside the tube, held with a nut) in each. Nuts on
   the bolts lower the centre of gravity. Start with four nuts on each.
5. Check the balance: with the beam level, it should return to level slowly when released.
   If it falls to one side, add nuts until it does not (or move a motor mount 1-2 mm).

## 4. Sensors

1. Fix the pot to `pot_bracket` (or a bent strip) on the outside of one upright, coaxial
   with the rod; join rod and pot shaft with `pot_coupler`. Turn the beam by hand: the pot
   must turn smoothly, without forcing the shaft sideways.
2. Mount the GY-521 with `imu_mount` on top of the beam **centred over the axle**, x axis
   (silk-screen arrow) along the beam towards motor 1.
3. Wire both to the Arduino (wiring.md). Run `s04_imu_check` with the motors unpowered and
   tilt the beam by hand: `pot`, `acc` and `comp` must agree in sign and roughly in value.
   If `acc` has the wrong sign, the board is rotated: fix the mounting, or change
   `BEAM_GYRO_SIGN` and the axes in `beam_imu.h`.

## 5. Stops, cage, power

1. Glue or screw rubber stops on the base under each end of the beam so it stops at
   about 30° (the tip of the beam about 150 mm lower than level).
2. Build the cage: a 900 x 450 x 600 mm (long x wide x high) timber frame of 25 x 25 mm
   strips (it must clear the guards with the beam at 30°), welded mesh
   stapled on all sides and the top. It drops over the clamped rig and stands on the
   bench; cut notches in its bottom rails for the G-clamps and the cables.
3. Mount the supply, fuse and e-stop on a separate board **outside** the cage. Wire the
   power chain (wiring.md). Leave the ESC red BEC wires disconnected and taped.

## 6. Commissioning (props off)

Follow lab 3, part A: ESC start-up beeps, arming test, each motor at 1150 µs, direction
check (motor 1 and 2 must spin so that their props, once fitted, push up), disarm tests
(switch off, pot unplugged, `x`). Then -- and only then -- fit props and guards.

Record the measured mass, the free-swing period and the thrust curve in the rig's log
sheet; notebook 3 turns them into the model parameters.
