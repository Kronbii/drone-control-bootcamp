#!/usr/bin/env sh
# Render the rig's OpenSCAD parts to STL (hardware/rig/parts/stl/).
# Needs OpenSCAD >= 2021.01 on the PATH (or OPENSCAD=/path/to/openscad).
set -e
OPENSCAD="${OPENSCAD:-openscad}"
cd "$(dirname "$0")/../hardware/rig/parts"
mkdir -p stl
for f in bearing_block pot_coupler pot_bracket motor_mount imu_mount prop_guard_segment; do
  echo "rendering $f"
  "$OPENSCAD" -o "stl/$f.stl" "$f.scad"
done
