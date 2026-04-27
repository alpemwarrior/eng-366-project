#pragma once 

/**
 * TODO: Implement your FSM here
 * 
 * This file implements a very basic FSM that initially lets the robot go forward
 * and stops it when an obstacle is detected.
 * 
 * You can modify this file to implement your own FSM and implement your own behaviors.
*/

#include "pioneer_interface/pioneer_interface.hpp" // contains the NUM_SENSORS constant 
#include "braitenberg.hpp" // you may want to use the braitenberg function to control the robot in some cases

#define THRESHOLD   1010  // Arbitrary threshold value for the proximity sensors


//////////////////////
// Global variables //
//////////////////////

// enum used to store the current behavior
enum basicBehaviors {GOFORWARD, STOP} behavior;

////////////////////////
// Behavior Functions //
////////////////////////

void goForwardBehavior(double* ps_values, double &vel_left, double &vel_right){
  vel_left = 1.0;
  vel_right = 1.0;
  return;
}

void stopBehavior(double &vel_left, double &vel_right){
  vel_left = 0;
  vel_right = 0;
  return;
}

///////////////////////
// Main FSM function //
///////////////////////

/**
 * @brief Finite State Machine that manages the robot's behavior
 * @param ps_values array of proximity values from the robot's proximity sensors
 * @param[out] vel_lef left wheels velocity
 * @param[out] vel_right right wheels velocity
*/
void fsm(double* ps_values, double &vel_left, double &vel_right){

  switch(behavior){

      case GOFORWARD:
        
        // Perform the goForward behavior
        goForwardBehavior(ps_values, vel_left, vel_right);

        // Check for transition criteria 
        for(int i=0; i<NUM_SENSORS; i++){
          if (ps_values[i]>THRESHOLD){
            printf("Obstacle detected, stopping...\n");
            behavior = STOP;
          }
        }
        break;

      case STOP:

        // Perform the stop behavior
        stopBehavior(vel_left, vel_right);

        // No transition criteria, the robot stays in this state forever
        break;

      default:
        printf("This behavior is not implemented.\n");
        break;
    }
    return;
}