// IMPLEMENTED BY: Pol Rius (423483)
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

#include "path_following.hpp" // path following
#include "signal_analysis.hpp"

//////////////////////
// Global variables //
//////////////////////

// enum used to store the current behavior
enum basicBehaviors {START, FOLLOW_PATH, STOP} behavior;

////////////////////////
// Behavior Functions //
////////////////////////
void stopBehavior(double &vel_left, double &vel_right) {
  vel_left = 0;
  vel_right = 0;
}

int followPathBehavior(double* ps, double* position, double &vel_left, double &vel_right) {
  double lws_p, rws_p, lws_b, rws_b;
  
  int wp = getWheelSpeeds(position, lws_p, rws_p);
  braitenberg(ps, lws_b, rws_b);
  vel_left = clipWheelSpeed(lws_p + lws_b);
  vel_right = clipWheelSpeed(rws_p + rws_b);
  return wp;
}

///////////////////////
// Main FSM function //
///////////////////////

double position[4]; // To be removed once odometry is implemented
int wp_last, wp;
double ds, dth;
LightAnalyzer lightLightSensor;

/**
 * @brief Finite State Machine that manages the robot's behavior
 * @param ps_values array of proximity values from the robot's proximity sensors
 * @param[out] vel_lef left wheels velocity
 * @param[out] vel_right right wheels velocity
*/
void fsm(double* ps_values, Pioneer* robot, double &vel_left, double &vel_right){

  switch(behavior){
  
      case START:                   
        printf("Starting FSM\n");
        kal_init();
        odom_init(robot);
        behavior = FOLLOW_PATH;
        wp_last = -1;
        // Do not break, execute FOLLOW_PATH immediately

      case FOLLOW_PATH:
        compute_odom(robot, &ds, &dth);
        kal_predict(ds, dth);

        position[0] = mu(0);
        position[1] = mu(1);
        position[2] = mu(2);
        position[3] = robot->get_time();



        robot->hide_state_estimate_marker();
        robot->set_state_estimate_marker(position[0], position[1], position[2], position[3]);

        //if (robot->get_ground_truth_pose(position)) {
        if (1) {
          lightLightSensor.update(robot, position[4], position[0], position[1]);

          // Follow path if location is available
          wp = followPathBehavior(ps_values, position, vel_left, vel_right);
          if (wp_last != wp) {
            printf("Following segment %d (waypoints %d -> %d)\n", wp, wp, wp+1);
            wp_last = wp;
          }

          // Transition criterion
          if (checkIfPathIsFinished(position)) {
            printf("Finished path following\n");
            behavior = STOP;
          }
        } else {
          printf("PRESS G\n");
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
