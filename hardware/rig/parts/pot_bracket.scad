// pot_bracket.scad -- L bracket that holds the pot body still, coaxial with the axle.
// Screw the foot to the outside of an upright; the pot's bushing goes through the hole,
// the anti-rotation tab of the pot into the small slot.
include <common.scad>

axle_height = 30;   // from the upright's surface to the axle centre (match your build)
foot_x = 40;
foot_y = 30;

union() {
  // foot on the upright
  difference() {
    plate(foot_x, foot_y, 4);
    for (sx = [-1, 1]) translate([sx * 14, 0, -1]) cylinder(d = 4.5, h = 6);
  }
  // vertical plate with the pot hole
  translate([-foot_x / 2, -2, 0])
    difference() {
      cube([foot_x, 4, axle_height + 15]);
      translate([foot_x / 2, -1, axle_height]) rotate([-90, 0, 0])
        cylinder(d = pot_bush_d + fit, h = 6);
      // anti-rotation tab slot, 8 mm from the shaft centre
      translate([foot_x / 2 + 8 - 1.2, -1, axle_height - 1.5]) cube([2.4, 6, 3]);
    }
}
