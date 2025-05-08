$fn = 200;  // High quality for resin printing

// Parameters
spline_diameter = 5.96;   // mm (outer diameter of 25T spline)
spline_depth = 5;         // mm
hex_size = 6.6;          // 1/4 inch hex (flat-to-flat)
hex_depth = 10;           // mm
wall_thickness = 1;       // mm between spline and hex
outer_diameter = 13;      // Slimmer than before

adapter_height = spline_depth + wall_thickness + hex_depth;

module hex_socket(size, depth) {
    rotate([0,0,30])
    linear_extrude(height = depth)
        polygon(points=[
            for(i = [0:5])
                [cos(i * 60) * size/2, sin(i * 60) * size/2]
        ]);
}

module servo_spline_socket(d, depth) {
    difference() {
        cylinder(h=depth, d=d+0.05);  // Add slight clearance
        for(i = [0:24]) {
            rotate([0,0,i*360/25])
                translate([d/2, 0, 0])
                    cylinder(h=depth, r=0.2);
        }
    }
}

module adapter() {
    difference() {
        cylinder(h=adapter_height, d=outer_diameter);

        // Spline cavity at base
        servo_spline_socket(spline_diameter, spline_depth);

        // Hex socket at top
        translate([0, 0, adapter_height - hex_depth])
            hex_socket(hex_size, hex_depth);
    }
}

adapter();
