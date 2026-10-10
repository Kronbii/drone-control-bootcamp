# Kit list and cost estimate

What one team (3 students) needs, what the cohort shares, and what it costs. The tables
are generated from [kit.csv](kit.csv) by `python tools/kit_cost.py --write`, so the
totals always add up; edit the CSV, not the tables.

## How these prices were estimated -- read before ordering

- They are **planning ranges in USD**, from typical listing prices on large online
  marketplaces and in electronics/hardware shops for these generic parts in 2025-2026.
  They are **not quotes**, and no order has been placed for this course yet.
- They **exclude shipping and customs**. Ordering to Lebanon from abroad typically takes
  weeks and adds shipping/customs; buying locally is faster and usually dearer. Budget a
  **25 % contingency** on top of the high column, and order motors, ESCs and props
  **at least 6 weeks** before session 3.
- Prices of the commodity parts (A2212 motors, 30 A ESCs, 1045 props, GY-521, Nano clones)
  move often; re-check them, and prefer sellers that state the ESC protocol (servo PWM)
  and the motor hole pattern.
- Space² is assumed to provide 3D printers, soldering stations, basic hand tools and
  space; the filament is costed. Students bring their own laptops.

## Tables

<!-- BEGIN:kit-tables -->
**Per team** (one bench kit and one rig):

| item | specification | qty | USD (low-high) | notes |
|---|---|---|---|---|
| Arduino Nano compatible | ATmega328P, CH340 USB; an Uno clone also works | 1 | 4-7 | one per team; students may bring their own |
| USB cable | matching the board | 1 | 1-2 | often included with the board |
| Breadboard and jumper wires | 830 points + 40 male-male wires | 1 | 3-5 |  |
| RC plant parts | 3 x 10 kOhm resistor, 3 x 100 uF 16 V electrolytic | 1 | 0.5-1.5 | labs 1-2 |
| Brushless motor | A2212 1000 KV outrunner (2212 920-1000 KV equivalents fine) | 2 | 12-20 | the common 'F450 kit' motor |
| ESC | 30 A, standard PWM input (BLHeli or SimonK firmware) | 2 | 12-20 | BEC output not used |
| Propeller pair | 10x4.5 (1045), one CW + one CCW | 2 | 3-6 | one pair fitted + one spare pair |
| Power supply | 12 V 10 A enclosed switching supply, mains cable | 1 | 12-20 | replaces a LiPo: no battery fire risk, current-limited |
| Emergency-stop switch | latching mushroom, >= 10 A, in the 12 V line | 1 | 4-8 | mandatory |
| Fuse holder and fuses | inline blade holder + 15 A fuses | 1 | 1-2 |  |
| Power wiring | 14 AWG silicone wire red/black 1 m each, XT60 pair, ring terminals | 1 | 4-6 |  |
| IMU module | MPU-6050 on a GY-521 board | 1 | 1.5-3 | session 4 |
| Potentiometer | 10 kOhm linear (B10K), 6 mm shaft | 1 | 0.5-1 | pivot angle sensor |
| Arm switch | toggle switch + leads | 1 | 0.5-1 |  |
| Beam | 20 x 20 x 1.5 mm aluminium square tube, 0.6 m | 1 | 3-6 | pine 20 x 20 mm also works |
| Axle hardware | M8 threaded rod 150 mm, 6 nuts, washers | 1 | 1-2 |  |
| Bearings | 608 (8 x 22 x 7 mm) | 2 | 0.8-2 | skateboard bearings |
| Plywood | 18 mm: base 400 x 250 mm, two uprights 320 x 80 mm | 1 | 5-10 |  |
| Fasteners | M3 bolts and nuts, wood screws (share of an assortment) | 1 | 2-4 |  |
| 3D-printing filament | PETG/PLA, about 350 g (blocks, coupler, brackets, mounts, 8 guard segments) | 1 | 6-9 | at 17-25 USD per kg |
| Safety cage | 12 mm welded mesh about 2 m2, 25 x 25 mm timber strips 8 m, staples (900 x 450 x 600 mm) | 1 | 14-25 | props-on work happens inside it |
| G-clamps | two 100 mm clamps | 2 | 5-8 | skip if the space has them |

**Shared per cohort:**

| item | specification | qty | USD (low-high) | notes |
|---|---|---|---|---|
| Safety glasses | one per student and instructor | 20 | 30-60 | mandatory when props are on |
| Kitchen scale | 1 g resolution, >= 3 kg | 2 | 16-24 | thrust curves in lab 3 |
| Spare motor | as above | 2 | 12-20 |  |
| Spare ESC | as above | 2 | 12-20 |  |
| Spare propeller pairs | as above | 6 | 9-18 | props break |
| Multimeter | basic digital | 2 | 16-30 | skip if the space has them |
| First-aid kit | with dressings for cuts | 1 | 10-20 |  |
| Fire extinguisher | CO2 or ABC powder | 1 | 20-35 | skip if the space has one |

**Totals** (USD, before shipping, customs and the contingency below):

| | low | high |
|---|---|---|
| bench kit per team (labs 1-2) | 8 | 16 |
| rig per team (labs 3-7) | 87 | 153 |
| **one team, total** | **96** | **168** |
| shared items per cohort | 125 | 227 |
| **cohort of 6 teams, one rig each** | **700** | **1,238** |
| lean cohort: 6 bench kits, 3 shared rigs | 438 | 779 |
<!-- END:kit-tables -->

## Notes

- **Lean option.** With three rigs for six teams, labs 3, 4 and 6 run in two shifts (half
  the teams on the rig, half on the simulation exercises, swapping at the break). That
  saves about a third of the cost but halves hardware time per team; see the
  [instructor guide](../instructor/guide.md#running-with-shared-rigs).
- **Reuse.** Everything except props, filament and some fasteners survives a cohort. A
  second cohort needs roughly the props, spares and consumables only.
- **Where the money goes.** The motors, ESCs, power supply and cage are about two thirds
  of a rig. Smaller motors (1104-1404 class with 3-inch props) cost similar money but need a
  new thrust curve and smaller guards; the course works unchanged once `RigParams` holds
  the new numbers.
- **Safety items are not optional**: e-stop, fuse, cage, glasses, first-aid kit and an
  extinguisher are part of the kit, not extras.
