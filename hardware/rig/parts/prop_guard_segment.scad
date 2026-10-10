// prop_guard_segment.scad -- one quarter of a ring guard around a 10-inch propeller.
// Print four per motor (eight per rig) and bolt them together at the tabs; two spokes
// (spoke = true) bolt to the motor mount. A guard does not make a spinning prop safe:
// the mesh cage, the e-stop and the rules in safety.md do. It stops casual contact and
// keeps a loose prop blade from flying out sideways.
include <common.scad>

prop_d    = 254;      // 10 inch
clearance = 12;       // radial gap between prop tips and the ring
ring_h    = 18;       // ring height
ring_t    = 3;        // ring thickness
spoke     = true;     // add a spoke to the motor mount on this segment

r_in = prop_d / 2 + clearance;

module tab() {
  difference() {
    cube([ring_t * 3, 10, ring_h], center = true);
    rotate([0, 90, 0]) cylinder(d = m3, h = 20, center = true);
  }
}

union() {
  // quarter ring
  rotate_extrude(angle = 90, $fn = 160)
    translate([r_in, 0]) square([ring_t, ring_h]);
  // joining tabs at both ends
  translate([r_in + ring_t / 2, 5, ring_h / 2]) tab();
  rotate([0, 0, 90]) translate([r_in + ring_t / 2, -5, ring_h / 2]) tab();
  // spoke from the ring to the motor mount (bolts to the mount's corner)
  if (spoke)
    rotate([0, 0, 45]) translate([30, -5, 0])
      difference() {
        cube([r_in - 30 + 1, 10, 4]);
        translate([5, 5, -1]) cylinder(d = m3, h = 6);
      }
}
