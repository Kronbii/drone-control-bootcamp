// imu_mount.scad -- puts a GY-521 (MPU-6050) board on the beam, centred over the pivot
// axis, with its x axis along the beam. Mounting holes of the common GY-521: two 3 mm
// holes 15.24 mm apart along one edge (measure yours).
include <common.scad>

hole_spacing = 15.24;
board_x = 21;     // along the beam
board_y = 16;
standoff_h = 5;
clamp_len = 24;

difference() {
  union() {
    // saddle on the beam
    translate([-clamp_len / 2, -(beam_w + 2 * wall) / 2, -(beam_w + wall)])
      cube([clamp_len, beam_w + 2 * wall, beam_w + 2 * wall]);
    // two standoffs for the board
    for (sx = [-1, 1]) translate([sx * hole_spacing / 2, board_y / 2 - 2.5, wall - 0.01])
      cylinder(d = 6, h = standoff_h);
  }
  translate([-clamp_len / 2 - 1, -(beam_w + fit) / 2, -(beam_w + wall) - 1])
    cube([clamp_len + 2, beam_w + fit, beam_w + wall + 1 - 0.01]);
  for (sx = [-1, 1]) translate([sx * hole_spacing / 2, board_y / 2 - 2.5, -1])
    cylinder(d = 2.6, h = standoff_h + wall + 2);  // M3 self-tapping
  // a cable tie through the saddle holds it on the beam
  translate([0, 0, -beam_w / 2]) cube([4, beam_w + 2 * wall + 2, 2], center = true);
}
