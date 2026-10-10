# The seesaw rig: one axis of a quadrotor on the bench

A 60 cm beam turns on an 8 mm axle in two skateboard bearings. A brushless motor with a
10-inch propeller sits at each end, 250 mm from the pivot. Pushing one motor up and the
other down rolls the beam -- exactly how a quadrotor rolls -- and the same cascaded
rate/angle controller holds it level. A potentiometer on the axle measures the true angle
(the ground truth that a real drone never has); an MPU-6050 on the pivot axis gives the
drone-like measurement.

```
           prop guard                                         prop guard
        .-----------.                                      .-----------.
       (   motor 1   )   IMU (on the axis)                (   motor 2   )
        '-----||----'          [_]                         '----||-----'
   ===========##===============(O)=========================##===========   beam 20x20 mm
              ^ +theta          |  axle, 608 bearings, pot on the end
                           |    |    |
                           |  upright|  x2, 18 mm plywood, axle 280 mm above the base
          stop            _|____|____|_            stop
   ======[==]============|  base, clamped to the bench |===========[==]=====
```

The rig runs inside a welded-mesh cage, powered from a current-limited 12 V supply through
an emergency-stop switch. **Read [safety.md](safety.md) before building or powering it.**

| document | what is in it |
|---|---|
| [bom.md](bom.md) | parts, specifications and alternatives (prices: [cohort/kit-and-costs.md](../../cohort/kit-and-costs.md)) |
| [build.md](build.md) | step-by-step mechanical build, with or without a 3D printer (about 3 h per rig) |
| [wiring.md](wiring.md) | power chain, signal wiring, grounding, pin map |
| [safety.md](safety.md) | the rules, the pre-power checklist, the roles at the bench |
| [parts/](parts/) | parametric OpenSCAD sources and rendered STL files |

## Design choices

- **Two motors, centre pivot.** One axis of a quad, including the mixer
  (`base ± u`) and the yaw-free "differential thrust" idea; a single-motor pendulum would
  teach less about drones.
- **A potentiometer on the axle.** Session 3 closes the loop on a clean, absolute angle
  before anyone has to think about IMUs; session 4 then *measures* how good the IMU
  estimate is against it. It is also the safety sensor (trip at ±28°).
- **Adjustable centre of gravity.** Nuts on two short bolts under the beam move the CG
  below the pivot (a stable pendulum, gentle first tests) or up to it (neutral, like a
  hovering drone). Notebook 3 finds the CG offset from the free-swing period.
- **Mains supply, not a LiPo.** A 12 V, 10 A enclosed supply has a hard current limit and no
  battery-fire risk; the rig never needs to move.
- **Common parts.** A2212 motors, 30 A ESCs and 1045 props are the standard "F450 kit"
  parts, available almost everywhere. Smaller motors and props work with new
  thrust-curve and inertia numbers in `bootcamp.rig.RigParams`.

## Numbers to measure on your rig (lab 3)

| quantity | how | model default (assumed) |
|---|---|---|
| thrust curve $T = c_1x + c_2x^2$ | kitchen scale under the other end, `s03_esc_bench` sweep | $c_1 = 1.6$ N, $c_2 = 6.4$ N |
| moving mass $m$ | kitchen scale, beam + motors + props + mounts | 0.29 kg |
| inertia $J$ | from the masses and distances: $J \approx 2 m_{motor\,end} l^2 + m_{beam} L^2/12$ | 0.0126 kg m² |
| CG below pivot $h$ | free-swing period, notebook 3 exercise 3.3 | 10 mm |
| friction $b$ | decay of the free swing (optional) | 0.010 N m s/rad |

These defaults are assumptions for this build and have not been measured on a built rig;
see [the pilot plan](../../cohort/pilot-plan.md).
