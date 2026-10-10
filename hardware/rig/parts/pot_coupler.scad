// pot_coupler.scad -- couples the 8 mm axle to the 6 mm pot shaft, so the pot reads
// the beam angle. Two M3 set screws (with nut traps) on each side. A short piece of
// silicone tube works too, but slips under load; this does not.
include <common.scad>

len_axle = 14;
len_pot  = 12;
od       = 18;

difference() {
  cylinder(d = od, h = len_axle + len_pot);
  translate([0, 0, -1]) cylinder(d = axle_d + fit, h = len_axle + 1);
  translate([0, 0, len_axle - 0.01]) cylinder(d = pot_shaft_d + fit / 2, h = len_pot + 1);
  for (z = [len_axle / 2, len_axle + len_pot / 2]) {
    // radial set-screw hole
    translate([0, 0, z]) rotate([0, 90, 0]) cylinder(d = m3 - 0.4, h = od);
    // nut trap, slot from the top of each section
    translate([od / 4 + 0.5, 0, z]) cube([2.8, m3_nut, 30], center = true);
  }
}
