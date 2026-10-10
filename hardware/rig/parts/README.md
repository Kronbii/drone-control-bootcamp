# 3D-printable parts

Parametric OpenSCAD sources; every dimension that depends on what you bought (beam size,
bearing, pot shaft, motor hole pattern, prop diameter) is a variable at the top of the
file or in `common.scad`. Measure your parts and edit before printing.

| file | qty per rig | purpose | print notes |
|---|---|---|---|
| `bearing_block.scad` | 2 | holds a 608 bearing on each upright | 4 walls, 40 % infill; press fit, sand if tight |
| `pot_coupler.scad` | 1 | 8 mm axle to 6 mm pot shaft, M3 set screws | print upright, 100 % infill |
| `pot_bracket.scad` | 1 | holds the pot coaxial with the axle | set `axle_height` to your build |
| `motor_mount.scad` | 2 | A2212 on the 20 mm beam, two M3 clamp bolts | PETG, 6 walls, 60 % infill |
| `imu_mount.scad` | 1 | GY-521 centred over the pivot axis | cable tie through the saddle |
| `prop_guard_segment.scad` | 8 | quarter ring around a 10-inch prop | 4 per guard; set `spoke = true` for 2 of them |

`stl/` holds the files rendered with OpenSCAD 2021.01 from the default parameters (all
render as closed, simple volumes). Re-render after changing a parameter:

```bash
tools/render_parts.sh            # needs openscad on the PATH, or OPENSCAD=/path/to/openscad
```

Material: PETG is preferred for the motor mounts and guards (tougher, does not shatter);
PLA is fine for the rest. About 350 g per rig in total.

These designs have been rendered but not yet printed and fitted on a real rig; check the
fits on the first print (see the [pilot plan](../../../cohort/pilot-plan.md)).
