// common.scad -- shared dimensions for the seesaw-rig parts (millimetres).
// Drone-control bootcamp, CC BY 4.0. Measure your own parts and adjust.

beam_w      = 20.0;   // square aluminium tube (or pine strip), outer size
fit         = 0.4;    // printing clearance for sliding fits
bearing_od  = 22.0;   // 608 bearing
bearing_w   = 7.0;
axle_d      = 8.0;    // M8 threaded rod or 8 mm steel rod
pot_shaft_d = 6.0;    // common B10K pot (WH148): 6 mm shaft
pot_bush_d  = 7.0;    // threaded bushing (M7)
m3          = 3.4;    // M3 clearance hole
m3_nut      = 6.4;    // M3 nut across flats (+ clearance)
wall        = 3.0;

$fn = 64;

// rounded rectangle plate, centred in x/y, sitting on z = 0
module plate(x, y, h, r = 3) {
  hull()
    for (sx = [-1, 1], sy = [-1, 1])
      translate([sx * (x / 2 - r), sy * (y / 2 - r), 0]) cylinder(r = r, h = h);
}
