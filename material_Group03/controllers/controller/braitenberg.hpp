#pragma once

#include "pioneer_interface/pioneer_interface.hpp"

#define SENSOR_NUM 16
#define SENSOR_MAX 1024

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

#define B1 -1
#define B2 -1
#define B3 -1
#define B4 -1

/*
[V] = [B][S]

Where [S] = [S0 ... S15]^T are the sensor readings (0 m -> 1024, >5 m -> 0)
      [B] = [B_0_0 ... B_0_15
             B_1_0 ... B_1_15] are the braitenberg coefficients
      [V] = [Vl, Vr]^T are the left and right wheel speeds

*/

double braitenberg_coefs[2][SENSOR_NUM] = {
    0, 0, 0, 0, B4, B3, B2, B1, 0, 0, 0, 0, 0, 0, 0, 0,
    B1, B2, B3, B4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
}


/**
 * @brief      This function implements the Braitenberg algorithm
 *              to control the robot's velocity.
 * @param      ps           Proximity sensor readings, normalized (NUM_SENSORS values)
 * @param      vel_left     The left velocity
 * @param      vel_right    The right velocity
*/
void braitenberg(double* ps, double &vel_left, double &vel_right){
    vel_left = 0.0; vel_right = 0.0;
    for (int i = 0; i < SENSOR_NUM; i++) {
        vel_left  += braitenberg_coefs[0][i]*ps[i]/SENSOR_MAX;
        vel_right += braitenberg_coefs[1][i]*ps[i]/SENSOR_MAX;
    }
}
