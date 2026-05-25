// Controller for the robot robot

#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"
  
#include "braitenberg.hpp"
#include "odometry.hpp"
//#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"
// #include "path_following.hpp"

int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  //WP3: LightSensor initialization (necessary even after integration)
  LightSensor lightLightSensor;

  // Initialize an example log file
  std::string f_example = "example.csv";
  int f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,");

  //WP3: Trajectory logging for offline plot (necessary even after integration)
  std::string f_traj = "trajectory.csv";
  int f_traj_cols = init_csv(f_traj, "time,x,y,");

  while (robot.step() != -1) {

    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////
   
    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity SensorLightSensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity();  // Light intensity
    double* imu = robot.get_imu();                // IMU with accelerations and rotation rates

    // ==================== WP3: LIGHT ANALYSIS ====================

    //WP3:(only for testing) Ground truth pose - can be commented when using keyboard controller
    double x = 0.0, y = 0.0;
    double pose[4] = {0.0, 0.0, 0.0, 0.0};

    bool got_pose = robot.get_ground_truth_pose(pose);

    if (got_pose) {
        x = pose[0];
        y = pose[1];
    }

    //WP3: Log trajectory for offline plot (necessary even after integration)
    log_csv(f_traj, f_traj_cols, time, x, y);

    //WP3: Call to LighhtLightSensor (necessary even after integration)
    lightLightSensor.update(robot, time, x, y);

    ////////////////////
    // Implementation //
    ////////////////////

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // NAVIGATION - WP1
    //(not required for WP3 testing, uncomment when integration)
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    fsm(ps_values, lws, rws);     // finite state machine 
    robot.set_motors_velocity(lws, rws); // set the wheel velocities

    // STATE ESTIMATION MARKER - WP2
    //(not required for WP3 testing, uncomment when integration)
    robot.hide_state_estimate_marker(); 
    robot.set_state_estimate_marker(0.0, 0.0, time, time);

    //////////////////
    // Data logging //
    //////////////////

    // Log the time and light and IMU data in a csv file 
    log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);
  }

  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}