#pragma once

#include "pioneer_interface/pioneer_interface.hpp"

/*

Pioneer sensor layout:

                Front
                3   4
        1   2           5   6
    0                           7
Left                             Right
    15                          8
        14  13          10  9
                12  11
                 Back

Response: >5m -> 0, 0m -> 1024
*/

/**
 * @brief      This function implements the Braitenberg algorithm
 *              to control the robot's velocity.
 * @param      ps           Proximity sensor readings (NUM_SENSORS values)
 * @param      vel_left     The left velocity
 * @param      vel_right    The right velocity
*/
void braitenberg(double* ps, double &vel_left, double &vel_right){

    // TODO: implement your Braitenberg algorithm here
}
