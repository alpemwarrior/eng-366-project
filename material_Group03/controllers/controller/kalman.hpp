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

// Process noise -> accounts for unmodeled physical effects, like wheel slip 
static const double KAL_SIGMA_XY_PER_M    = 0.05;  // [m/m]   positional slip
static const double KAL_SIGMA_THETA_PER_M = 0.03;  // [rad/m] heading slip

// Measurement noise
// GYR_STD given in pioneer_interface.hpp

// Wall pseudo-measurement noise [m] — tight because wall geometry is known exactly
static const double KAL_SIGMA_WALL  = 0.05;        // [m]
 
// Only fire a wall update when the relevant sonar reads below this [m]
static const double WALL_SONAR_THRESH = 0.6;
 

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
    sigma(0,0) = 0.01 * 0.01;   // ~1 cm std-dev on x
    sigma(1,1) = 0.01 * 0.01;   // ~1 cm std-dev on y
    sigma(2,2) = 0.001 * 0.001; // ~0.06 deg std-dev on heading
}

// kal_predict: implements prediction step of EKF algorithm 
void kal_predict(double ds, double dtheta) {
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
    static double gyro_heading = 0.0;
    static bool   gyro_init    = false;

    // on first call, initialise gyro and gyro_heading 
    if(!gyro_init) { 
        gyro_heading = 0.0; 
        gyro_init = true; 
    }

    // add measured change in heading to gyro_heading
    gyro_heading = kal_wrap(gyro_heading + z_meas);
    
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
    Eigen::Matrix<double,3,1> K_t = sigma * H_t.transpose() * (H_t * sigma * H_t.transpose() + Q_t).inverse();
    
    // Step 4 of EKF algorithm 
    mu = mu + K_t * kal_wrap(gyro_heading - mu(2));
    mu(2) = kal_wrap(mu(2));

    // Step 5 of EKF algorithm 
    sigma = (I - K_t * H_t) * sigma;
    
    // check for NaN values
    if(kal_check_nan(sigma)) sigma = Mat::Identity() * 0.01;
}

// kal_node_correction: additional correction, robot receives x and y location of each node via serial communication
void kal_node_correction(double node_x, double node_y, double signal_strength) {
    // check if message has been received 
    if (signal_strength <= 0.0) {
        return;
    }

    
}

// kal_wall_correction: additional correction, as we know wall positions in x relative to start position
void kal_wall_correction(double wall_x, double sonar_dist){
    // check if close to wall
    if(sonar_dist > WALL_SONAR_THRESH) {
        return;
    }

    // Observation model Jacobian 
    Eigen::Matrix<double,1,3> H_t;
    H_t(0,0) = 1.0; 
    H_t(0,1) = 0.0; 
    H_t(0,2) = 0.0;
 
    // measurement noise matrix 
    Eigen::Matrix<double,1,1> Q_t;
    Q_t(0,0) = KAL_SIGMA_WALL * KAL_SIGMA_WALL;
 
 
    // Step 3
    Eigen::Matrix<double,3,1> K_t = sigma * H_t.transpose() * (H_t * sigma * H_t.transpose() + Q_t).inverse();
    
    // Step 4
    mu    = mu + K_t * (wall_x - mu(0));
    mu(2) = kal_wrap(mu(2));

    // Step 5
    sigma = (I - K_t * H_t) * sigma;
    
    // check for NaN
    if(kal_check_nan(sigma)) sigma = Mat::Identity() * 0.01;
}
