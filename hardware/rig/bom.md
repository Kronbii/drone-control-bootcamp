# Bill of materials (one rig)

Prices, the shared items and cohort totals are in
[cohort/kit-and-costs.md](../../cohort/kit-and-costs.md), generated from
[cohort/kit.csv](../../cohort/kit.csv). This page is about *what* to buy and *why*.

## Electronics and power

| qty | part | specification | notes / alternatives |
|---|---|---|---|
| 1 | Microcontroller | Arduino Nano (ATmega328P) or Uno | The sketches use Timer1 on pins 9/10 and therefore need an ATmega328P board (Uno, Nano, Pro Mini). |
| 2 | Brushless motor | A2212, 1000 KV | Any 2212-2216 outrunner of 900-1100 KV. Check the mount hole pattern (16/19 mm) against `parts/motor_mount.scad`. |
| 2 | ESC | 30 A, standard servo-PWM input | BLHeli or SimonK. Must accept 1000-2000 µs pulses at 50-400 Hz (nearly all do). The 5 V BEC output is **not** used. |
| 2+2 | Propellers | 10x4.5 (1045), one CW and one CCW per rig, plus a spare pair | Plastic, not carbon. Mount so that each pushes its end **up**. |
| 1 | Power supply | 12 V, 10 A, enclosed metal case | Mains-side terminals must be covered. A bench supply with current limit is even better. |
| 1 | Emergency stop | Latching mushroom switch, ≥ 10 A DC at 12 V | Breaks the 12 V line to both ESCs. Mounted outside the cage, within arm's reach of the operator. |
| 1 | Fuse | Inline blade fuse holder, 15 A fuse | Directly after the supply's + terminal. |
| 1 | Power wiring | 14 AWG silicone, XT60 pair, ring terminals | |
| 1 | IMU | MPU-6050 on a GY-521 board | 5 V tolerant (has a regulator); I²C on A4/A5. |
| 1 | Potentiometer | 10 kΩ linear (B10K), 6 mm shaft, M7 bushing | A smooth, single-turn carbon pot is fine; avoid detented pots. |
| 1 | Arm switch | SPST toggle | D2 to GND. |
| - | Wires, header, heat-shrink | | |

## Mechanics

| qty | part | specification | notes |
|---|---|---|---|
| 1 | Beam | 20 x 20 x 1.5 mm aluminium square tube, 600 mm | Pine 20 x 20 mm works (heavier, softer: re-measure $J$). Drill an 8.5 mm hole through the exact centre. |
| 1 | Axle | M8 threaded rod, 150 mm, 6 nuts, 4 washers | The beam is clamped to the axle between two nut pairs, so the axle turns with it. |
| 2 | Bearings | 608 (8 x 22 x 7 mm) | Press-fit into 22 mm holes in the uprights or into `bearing_block` prints. |
| 1 | Base | 18 mm plywood, 400 x 250 mm | Clamped to the bench with two G-clamps. |
| 2 | Uprights | 18 mm plywood, 320 x 80 mm | Axle centre 280 mm above the base. |
| 2 | Stops | Rubber door stops or dense foam blocks | Under the beam ends, so the beam stops at about ±30°. |
| 2 | CG adjusters | M6 x 60 mm bolts with 4-6 nuts each | Hang below the beam, 30 mm either side of the axle. |
| 1 | Cage | 12 mm welded mesh (about 2 m²) on a 25 x 25 mm timber frame (about 8 m), 900 x 450 x 600 mm, open bottom | All props-on work happens inside it; it must clear the guards at ±30°. |
| - | Fasteners | M3 bolts/nuts, 4 mm wood screws, cable ties | |

## 3D-printed parts (optional but recommended)

See [parts/README.md](parts/README.md). About 350 g of PETG (preferred: tougher near
spinning parts) or PLA per rig.

| qty | part | replaces |
|---|---|---|
| 2 | `bearing_block` | drilling 22 mm holes in the uprights |
| 1 | `pot_coupler` | a piece of silicone tube (slips) |
| 1 | `pot_bracket` | a bent metal strip |
| 2 | `motor_mount` | a wooden block and hose clamps |
| 1 | `imu_mount` | double-sided foam tape on the beam |
| 8 | `prop_guard_segment` (2 per guard with spoke) | nothing: guards are part of the safety chain |

## What not to use

- **LiPo batteries**: no need on a fixed rig, and they add a fire risk to a student lab.
- **Carbon propellers**: stiffer and sharper; plastic props break instead of cutting deeper.
- **The ESC's BEC to power the Arduino** while it is also on USB: two 5 V sources fighting.
