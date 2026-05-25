// Controller for the robot robot

// Provided libraries 
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"
  
#include "braitenberg.hpp"
#include "odometry.hpp"
//#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"
//#include "path_following.hpp"

int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  //WP1: Temperature logging (not required for WP3)
  // Initialize temperature .csv
  std::string f_temperature = "temperature.csv";
  int f_temperature_cols = init_csv(f_temperature, "time, sensor_id, sensor_x, sensor_y, temp_in, temp_out,");
  // <-- don't forget the comma at the end of the string!!

  // Initialize an example log file
  std::string f_example = "example.csv";
  int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); 
  // <-- don't forget the comma at the end of the string!!

  //WP3: Trajectory logging for offline plot (necessary even after integration)
  //std::string f_traj = "trajectory.csv";
  //int f_traj_cols = init_csv(f_traj, "time,x,y,");

  while (robot.step() != -1) {

    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////
   
    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity Sensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity();  // Light intensity
    double* imu = robot.get_imu();                // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    //WP3: Log trajectory for offline plot (necessary even after integration)
    //log_csv(f_traj, f_traj_cols, time, x, y);


    ////////////////////
    // Implementation //
    ////////////////////

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // NAVIGATION - WP1
    //(not required for WP3 testing, uncomment when integration)
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    
    fsm(ps_values, &robot, lws, rws);     // finite state machine 
    

    robot.set_motors_velocity(lws, rws); // set the wheel velocities

    // STATE ESTIMATION MARKER (green arrow in simulation) - WP2
    //(not required for WP3 testing, uncomment when integration)
    robot.hide_state_estimate_marker(); // this hides the marker in the simulation
    robot.set_state_estimate_marker(0.0, 0.0, time, time); // rotate in place for now, input your state estimate here for visualization in the simulation!

    //////////////////
    // Data logging //
    //////////////////

    // Log the time and light and IMU data in a csv file 
    log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);

    if (signal_strength != 0.0) {
      kal_node_correction(data[1], data[2], signal_strength);
      log_csv(f_temperature, f_temperature_cols, time, data[0], data[1], data[2], data[3], data[4]);
    }

  }

  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}
