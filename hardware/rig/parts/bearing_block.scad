// bearing_block.scad -- holds one 608 bearing; screw two blocks to the uprights
// (an alternative to drilling 22 mm holes in the plywood with a Forstner bit).
include <common.scad>

block_x = 46;   // width
block_y = 46;   // height on the upright
block_t = bearing_w + 3;

difference() {
  plate(block_x, block_y, block_t);
  // bearing pocket, press fit (no clearance on purpose: sand if too tight)
  translate([0, 0, 3]) cylinder(d = bearing_od, h = block_t);
  // through hole for the axle, with room for it to turn
  translate([0, 0, -1]) cylinder(d = axle_d + 3, h = block_t + 2);
  // four wood-screw holes (4 mm screws)
  for (sx = [-1, 1], sy = [-1, 1])
    translate([sx * 17, sy * 17, -1]) cylinder(d = 4.5, h = block_t + 2);
}
