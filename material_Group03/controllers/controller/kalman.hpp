// Implemented by: Charles Bishop, 423149

#include <math.h>
#include <memory.h>

#include "odometry.hpp"

///////////////////
// Eigen library //     DO NOT MODIFY THIS PART
///////////////////

#include <Eigen/Dense>

#define DIM 3                                       // State dimension 

typedef Eigen::Matrix<double,DIM,DIM>   Mat;        // DIMxDIM matrix  
typedef Eigen::Matrix<double, -1, -1>   MatX;       // Arbitrary size matrix 
typedef Eigen::Matrix<double,DIM,  1>   Vec;        // DIMx1 column vector  
typedef Eigen::Matrix<double, -1,  1>   VecX;       // Arbitrary size column vector  

static const Mat I = MatX::Identity(DIM,DIM);       // DIMxDIM identity matrix  

//////////////////////////////////
// Kalman filter base functions //    DO NOT MODIFY THIS PART
//////////////////////////////////

// State vector mu (x,y,heading) to be updated by the Kalman filter functions
static Vec mu = Vec::Zero();
// State covariance sigma to be updated by the Kalman filter functions
static Mat sigma = Mat::Zero();

/**
 * @brief      Get the state dimension 
*/
int kal_get_dim(){
    return DIM;
}

/**
 * @brief      Copy the state vector into a 1D array
*/
void kal_get_state(double* state){
    for(int i=0; i<DIM; i++){
        state[i] = mu(i);
    }
}

/**
 * @brief      Copy the state covariance matrix into a 2D array
*/
void kal_get_state_covariance(double** cov){
    for(int i=0;i<sigma.rows();i++){
        for(int j=0; j<sigma.cols(); j++){
            cov[i][j] = sigma(i,j);
        }
    }
}

/**
 * @brief      Check if a matrix contains any NaN values 
*/
bool kal_check_nan(const MatX& m){
    for(int i=0;i<m.rows();i++){
        for(int j=0; j<m.cols(); j++){
            if(isnan(m(i,j))){
                printf("FATAL: matrix has NaN values, exiting...\n");
                return true;
            }
        }
    }
    return false;
}

///////////////////////////////////////////////////
// TODO: implement your Kalman filter here after //
///////////////////////////////////////////////////

// *********************
// ----- CONSTANTS -----
// *********************

// track last heading
static double last_mu_theta = 0.0;

// Process noise -> accounts for unmodeled physical effects, like wheel slip 
static const double KAL_SIGMA_XY_PER_M = 0.2;  // [m/m]   positional slip
static const double KAL_SIGMA_THETA_PER_M = 0.1;  // [rad/m] heading slip

// Measurement noise
// GYR_STD given in pioneer_interface.hpp

// Wall pseudo-measurement noise [m] — tight because wall geometry is known exactly
static const double KAL_SIGMA_WALL  = 0.05;        // [m]
 
// Only fire a wall update when the relevant sonar reads below this [m]
static const double WALL_SONAR_THRESH = 0.6;
 
// Node geometry 
static const double NODE_HEIGHT    = 1.000;                          
static const double ROBOT_SENSOR_H = PioneerInfo::height;            
static const double DELTA_H        = NODE_HEIGHT - ROBOT_SENSOR_H;  

// Constants relating to wall heading correction
#define WALL_CORRECTION_THRESHOLD 1.0
#define ANGLE_DIF (2*M_PI/9)
#define PS_TO_DIST(x) ((1024.0 - x)*5.0/1024.0)
#define KAL_SIGMA_WALL_HEADING 0.05

// *********************
// ----- FUNCTIONS -----
// *********************

// helper function to normalise angles to [-pi, pi]
static inline double normalize_angle(double angle) {
    while(angle > M_PI)  angle -= 2.0 * M_PI;
    while(angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// function to initialise Kalman filter (state vector and covariance matrix)
void kal_init() {
    mu  = Vec::Zero();
    sigma = Mat::Zero();

    // small initial uncertainty, robot starts at origin facing x axis
    sigma(0,0) = 1e-6;   
    sigma(1,1) = 1e-6;   
    sigma(2,2) = 1e-6; 
}

// kal_predict: implements prediction step of EKF algorithm 
void kal_predict(double ds, double dtheta) {
    // save mu_theta for next function call
    last_mu_theta = mu(2);

    // use midpoint heading for prediction
    double theta_mid = mu(2) + dtheta * 0.5; 

    // estimate state vector 
    // equivalent to: x^hat_t = f(x_{t-1}, u_t) from week 11 lecture slides
    mu(0) += ds * cos(theta_mid);
    mu(1) += ds * sin(theta_mid);
    mu(2) = normalize_angle(mu(2) + dtheta);

    // compute Jacobian of motion model 
    Mat F_t = I; 
    F_t(0,2) = -ds * sin(theta_mid);
    F_t(1,2) =  ds * cos(theta_mid);

    // estimate process noise
    double abs_ds = fabs(ds);
    double r_xy = KAL_SIGMA_XY_PER_M * abs_ds + 1e-6;           // add small term to prevent zero noise when ds is zero
    double r_theta = KAL_SIGMA_THETA_PER_M * abs_ds + 1e-6;

    // process noise matrix 
    Mat R_t =  Mat::Zero();
    R_t(0,0) = r_xy * r_xy;
    R_t(1,1) = r_xy * r_xy;
    R_t(2,2) = r_theta * r_theta;

    // calculate predicted covariance 
    sigma = F_t * sigma * F_t.transpose() + R_t;

    // check for NaN values, which can cause Kalman filter to diverge 
    if(kal_check_nan(sigma)) {
        sigma = Mat::Identity() * 0.01;
    }
}

// kal_gyro_correction: performs correction stage of EKF algorithm, using gyro readings to correct for heading drift in the odometry
void kal_gyro_correction(double gyro_z, double dt) {
    // measured change in heading 
    double z_meas  = gyro_z * dt;

    // static variables to track gyro-integrated heading across function calls
    static double last_mu_theta = 0.0;
    static bool gyro_init = false;

    // on first call, initialise gyro and gyro_heading 
    if(!gyro_init) { 
        last_mu_theta = mu(2);
        gyro_init = true; 
        return;
    }

    // predicted change in heading from odometry
    double z_pred = normalize_angle(mu(2) - last_mu_theta);
    
    // Jacobian of observation model
    Eigen::Matrix<double, 1, 3> H_t;
    H_t(0,0) = 0.0; 
    H_t(0,1) = 0.0; 
    H_t(0,2) = 1.0;
    
    // measurement noise matrix 
    Eigen::Matrix<double,1,1> Q_t;
    double sigma_z = GYR_STD * dt;
    Q_t(0,0) = sigma_z * sigma_z;
    
    // Step 3 of EKF algorithm
    Eigen::Matrix<double, 3, 1> K_t = sigma * H_t.transpose() * (H_t * sigma * H_t.transpose() + Q_t).inverse();
    
    // Step 4 of EKF algorithm 
    mu = mu + K_t * normalize_angle(z_meas - z_pred);
    mu(2) = normalize_angle(mu(2));

    // Step 5 of EKF algorithm 
    sigma = (I - K_t * H_t) * sigma;
    
    // save mu_theta for next function call
    last_mu_theta = mu(2);

    // check for NaN values
    if(kal_check_nan(sigma)) {
        sigma = Mat::Identity() * 0.01;
    }
}

// kal_node_correction: additional correction, robot receives x and y location of each node via serial communication
void kal_node_correction(double node_x, double node_y, double signal_strength) {
    // check if message has been received 
    if (signal_strength <= 0.0) {
        return;
    }

    double s_squared = 1 / signal_strength; 
    double dh_squared = DELTA_H * DELTA_H;

    // return if robot under the node
     if (s_squared <= dh_squared + 1e-9) {
        return;
    }

    // displacement in xy plane from robot to node 
    double s_xy_measured = sqrt(s_squared - dh_squared);

    // predicted distance from robot to node
    double dx = node_x - mu(0);
    double dy = node_y - mu(1);
    double s_xy_predicted = sqrt(dx*dx + dy*dy);

    if (s_xy_predicted < 1e-6) {
        return;  // avoid division by zero
    }

    // Jacobian of observation model h(x) = sqrt((x_n-x)^2 + (y_n-y)^2)
    Eigen::Matrix<double, 1,3> H_t;
    H_t(0,0) = -dx / s_xy_predicted;  // dh/dx
    H_t(0,1) = -dy / s_xy_predicted;  // dh/dy
    H_t(0,2) =  0.0;

    // measurement noise matrix 
    static const double KAL_SIGMA_RANGE = 0.15;  // [m]
    Eigen::Matrix<double, 1, 1> Q_t;
    Q_t(0,0) = KAL_SIGMA_RANGE * KAL_SIGMA_RANGE;


    // Step 3
    Eigen::Matrix<double, 3, 1> K_t = sigma * H_t.transpose() * (H_t * sigma * H_t.transpose() + Q_t).inverse();

    // Step 4
    mu    = mu + K_t * (s_xy_measured - s_xy_predicted);
    mu(2) = normalize_angle(mu(2));

    // Step 5
    sigma = (I - K_t * H_t) * sigma;

    if(kal_check_nan(sigma)) {
        sigma = Mat::Identity() * 0.01;
    }
}

// getWallHeadingError: uses sonar readings when near a wall to estimate angle of robot relative to that wall
bool getWallHeadingError(Pioneer* robot, double* rheading) {
    // check if EKF estimate places robot near a known wall
    bool near_front = fabs(mu(0) - 7.0)  < WALL_CORRECTION_THRESHOLD;
    bool near_back  = fabs(mu(0) - (-1.3)) < WALL_CORRECTION_THRESHOLD;
    double r1, r2, alpha, c;
    double* ps;

    // if not near a known wall, then no estimation is possible
    if(!near_front && !near_back) {
        return false;
    }

    // correct applied when robot roughly turned alogside the wall (heading ≈ π/2), tolerance rejects corrections mid-turn
    if (fabs(mu(2) - M_PI_2) > (M_PI/6)) {
        return false;
    }

    ps = robot->get_proximity();
    
    if (near_front) {
        // r1, r2: get distances from sonar
        r1 = PS_TO_DIST(ps[6]); 
        r2 = PS_TO_DIST(ps[9]);

        // cosine rule: find length c of triangle formed by r1, r2 and the wall, given that sonars are separated by angle 2*ANGLE_DIF
        c = sqrt(r1*r1 + r2*r2 - 2*r1*r2*cos(2*ANGLE_DIF));

        // sine rule: find alpha, the angle between r1 and the wall normal
        alpha = asin(r1*sin(2*ANGLE_DIF)/c);

        // convert alpha to absolute heading estimate
        *rheading = -M_PI_2 + alpha + ANGLE_DIF;
    }

    return true;
}

// kal_wall_heading_correction: additional correction, using estimate of angle of robot relative to wall
void kal_wall_heading_correction(Pioneer* robot) {
    // check if EKF estimate places robot near a known wall
    double measured_heading;

    // if not near a wall, return 
    if (!getWallHeadingError(robot, &measured_heading)) {
        return;
    }

    // getWallHeadingError returns angle relative to wall, so add pi/2 to convert robot heading frame
    measured_heading += M_PI_2;

    // innovation: difference between measured and predicted heading
    double heading_err = normalize_angle(measured_heading - mu(2));

    // observation model 
    Eigen::Matrix<double,1,3> H_t;
    H_t(0,0) = 0.0; H_t(0,1) = 0.0; H_t(0,2) = 1.0;

    // measurement model
    Eigen::Matrix<double,1,1> Q_t;
    Q_t(0,0) = KAL_SIGMA_WALL_HEADING * KAL_SIGMA_WALL_HEADING;

    // step 3
    Eigen::Matrix<double,3,1> K_t = sigma * H_t.transpose() * (H_t * sigma * H_t.transpose() + Q_t).inverse();

    // step 4
    mu = mu + K_t * heading_err;
    mu(2) = normalize_angle(mu(2));

    // step 5
    sigma = (I - K_t * H_t) * sigma;

    // check for NaN values 
    if (kal_check_nan(sigma)) {
        sigma = Mat::Identity() * 0.01;
    }
}
