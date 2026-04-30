// Controller for the robot robot

// Provided libraries 
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"

// Files to implement your solutions  
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"
#include "path_following.hpp"

int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  // Initialize an example log file
  std::string f_example = "example.csv";
  int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!

  while (robot.step() != -1) {

    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////
    
    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity sensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity();  // Light intensity
    double* imu = robot.get_imu();                // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    ////////////////////
    // Implementation //
    ////////////////////

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // NAVIGATION
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    vec_2d wheel_speeds = vec_2d(0, 0);
    double position[4];
    
    if( robot.get_ground_truth_pose(position) ) {
      wheel_speeds = getWheelSpeeds(vec_2d(position[0], position[1]), vec_2d(cos(position[2]), sin(position[2])));
    } else {
      printf("Ground truth is disabled, press 'G'");
    }
    
    //fsm(ps_values, lws, rws);     // finite state machine 
    robot.set_motors_velocity(wheel_speeds.x()*pioneer_info.wheel_radius, wheel_speeds.y()*pioneer_info.wheel_radius); // set the wheel velocities

    // STATE ESTIMATION MARKER (green arrow in simulation)
    robot.hide_state_estimate_marker(); // this hides the marker in the simulation
    robot.set_state_estimate_marker(0.0, 0.0, time, time); // rotate in place for now, input your state estimate here for visualization in the simulation!

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
