// Implemented by: Charles Bishop, 423149

#pragma once 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"

#define RAD2DEG(X)      X / M_PI * 180.0 // Convert radians to degrees

// TODO: You can implement your wheel odometry here if relevant for your project

// variables to track previous wheel encoder values, and if odometry has been intialised
static double odom_prev_left  = 0.0;
static double odom_prev_right = 0.0;
static bool   odom_initialised = false;

// init_odom: called once after robot interface ready to initialise odometry
void odom_init(Pioneer* robot) {
    double* enc    = robot->get_encoders();
    odom_prev_left  = enc[0];
    odom_prev_right = enc[1];
    odom_initialised = true;
}

// compute_odom: called periodically to update odometry estimates
void compute_odom(Pioneer* robot, double* ds_out, double* dth_out) {
    // check odometry has been initialised, if not, initialise and set velocities to zero
    if(!odom_initialised){
        odom_init(robot);
        *ds_out  = 0.0;
        *dth_out = 0.0;
        return;
    }

    // get current wheel encoder values
    double* enc = robot->get_encoders();
    double current_left  = enc[0];
    double current_right = enc[1];

    // find linear displacements of each wheel (s = r * delta_theta)
    double ds_left  = pioneer_info.wheel_radius * (current_left  - odom_prev_left);
    double ds_right = pioneer_info.wheel_radius * (current_right - odom_prev_right);
    
    // compute linear displacement: average of two wheels 
    *ds_out  = (ds_right + ds_left)  * 0.5;

    // compute angular displacement: difference divided by axis length
    *dth_out = (ds_right - ds_left)  / pioneer_info.axis_length;
 
    // update previous encoder values for next function call 
    odom_prev_left  = current_left;
    odom_prev_right = current_right;
}