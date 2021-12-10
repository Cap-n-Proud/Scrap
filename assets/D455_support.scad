$fn=200;

//Support for Intel Realsense camea D455
realsense_H = 29;
realsense_L = 124;
realsense_hole_space=95;
realsense_screw_M=4+1.5;
base_thickness = 6;
    
module SUB_realsense_holes(){
    translate([0, 0, 0])cylinder(h=50,d=realsense_screw_M, center = true);
    translate([realsense_hole_space/2, 0, 0])cylinder(h=50,d=realsense_screw_M, center = true);
    translate([-realsense_hole_space/2, 0, 0])cylinder(h=50,d=realsense_screw_M, center = true);
    
    //Creates bigger holes to house the head of the screw
    translate([realsense_hole_space/2, 0, -25/2])cylinder(h=25,d=2*realsense_screw_M, center = true);
    translate([-realsense_hole_space/2, 0, -25/2])cylinder(h=25,d=2*realsense_screw_M, center = true);
}

module realsense_support() {
    
    //translate([-realsense_L/2, 0, 0])cube([realsense_L, 5, 5], center = false);
    difference(){
        
    translate([-realsense_L / 4-realsense_H/2, 0, 0]) linear_extrude(height = base_thickness, center = true, convexity = 10, twist = 0) hull() {
        translate([realsense_L - 1 * realsense_H, 0, 0]) circle(d = realsense_H);
        circle(d = realsense_H);
    }
    SUB_realsense_holes();
    //Create a depression to allow fixing the base to the frame of the robot
    translate([0, 0, realsense_H/2])cube([0.7*(realsense_L-realsense_H), realsense_H, realsense_H], center = true);
    
    }
}
realsense_support();
//SUB_realsense_holes();
