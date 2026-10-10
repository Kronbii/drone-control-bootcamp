// motor_mount.scad -- clamps a 2212/2216-class outrunner to the 20 mm square beam.
// Hole patterns: A2212 motors usually have M3 holes on 16 mm and 19 mm diagonals;
// set motor_holes to what yours has. Print two (100 % infill near the holes, PETG
// preferred), and fit them so the motor axes are vertical and 250 mm from the pivot.
include <common.scad>

motor_holes = [16, 19];   // hole-circle diameters (mm), cross pattern
base_x = 44;
base_y = 44;
base_t = 5;
clamp_h = beam_w + 2 * wall;

difference() {
  union() {
    plate(base_x, base_y, base_t);
    // saddle that wraps three sides of the beam, under the plate
    translate([-(beam_w + 2 * wall) / 2, -base_y / 2, -clamp_h + 0.01])
      cube([beam_w + 2 * wall, base_y, clamp_h]);
  }
  // beam channel (open at the bottom)
  translate([-(beam_w + fit) / 2, -base_y / 2 - 1, -clamp_h - 1])
    cube([beam_w + fit, base_y + 2, clamp_h + 1 - 0.01]);
  // motor holes: one pair on each diagonal circle, at 90 degrees
  for (i = [0:1]) for (a = [0, 180])
    rotate([0, 0, a + 90 * i + 45]) translate([motor_holes[i] / 2, 0, -1])
      cylinder(d = m3, h = base_t + 2);
  // clearance for the motor's shaft circlip under the plate centre
  translate([0, 0, -1]) cylinder(d = 9, h = base_t + 2);
  // two M3 cross bolts through the saddle clamp the beam
  for (y = [-12, 12]) translate([0, y, -clamp_h / 2]) rotate([0, 90, 0])
    cylinder(d = m3, h = beam_w + 4 * wall, center = true);
}
