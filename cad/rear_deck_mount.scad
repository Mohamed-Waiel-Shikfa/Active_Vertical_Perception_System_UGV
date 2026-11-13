// Millimetres. Prototype; verify the installed lid/PACS plate and rail before print.
// Bare Jackal lid: 120 mm square, M5x0.8. PACS 026369: 80 mm square.
deck_hole_pitch = 120;
print_clearance = 0.4;
rail_hole_pitch = 17;
rail_heights = [28, 166];

module gusset(y) {
  translate([-60, y, 8])
    rotate([90,0,0])
      linear_extrude(height=6)
        polygon([[0,0],[55,0],[0,154]]);
}
module rear_deck_mount() {
  difference() {
    union() {
      translate([-75,-75,0]) cube([150,150,8]);
      translate([-60,-22,8]) cube([10,44,190]);
      gusset(25);
      gusset(-19);
    }
    for (x=[-1,1], y=[-1,1])
      translate([x*deck_hole_pitch/2-6, y*deck_hole_pitch/2-(5+print_clearance)/2, -1])
        cube([12,5+print_clearance,10]);
    for (y=[-1,1], z=rail_heights)
      translate([-61,y*rail_hole_pitch/2-(4+print_clearance)/2,z-(4+print_clearance)/2])
        cube([12,4+print_clearance,4+print_clearance]);
  }
}
rear_deck_mount();
