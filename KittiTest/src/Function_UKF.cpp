#include "UKF.h"
/* \author Leo Wang & Varun Hegde*/
// Customized Supporting function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */

// Set process noise covariance matrix Q [10, 10]
void 
 UKF::SetProcessNoiseCovatiance(const float dt, const float sGPS, const float sCourse, 
                                 const float sTurnRate){
   Eigen::MatrixXf i = Eigen::MatrixXf::Identity(10, 10);  
   Eigen::DiagonalMatrix<float, 10> m;
   m.diagonal() << 0.5 * sGPS * dt * dt, 0.5 * sGPS * dt * dt, 0.5 * sGPS * dt * dt, sGPS * dt, 
                    sCourse * dt, sCourse * dt, sCourse * dt, sTurnRate * dt, sTurnRate * dt, sTurnRate * dt;
   Q_ = i * m;
}

// Set measurement noise covariance R [7, 7]
void 
 UKF::SetMeasureNoiseCovatiance(const float var_GPS, const float var_speed, const float var_turn_angle){
   Eigen::MatrixXf i = Eigen::MatrixXf::Identity(7, 7);  
   Eigen::DiagonalMatrix<float, 7> m;
   m.diagonal() << var_GPS * var_GPS, var_GPS * var_GPS, var_GPS * var_GPS, var_speed * var_speed,
                   var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle;
   R_ = i * m;
}

// Set initial uncertainty P0 [15, 15]
void 
 UKF::SetInitialCovariance(){
   Eigen::MatrixXf i = Eigen::MatrixXf::Identity(15, 15);  
   P0_ = i * 1000.0f;
}

// Initialize state x_pre = X0 and p_pre = P0 
void 
 UKF::Initialize(const Odometer &odom, const Oxts_Data &oxts_data){
   // UKF Initialize noise covariance
   SetProcessNoiseCovatiance(UKF::dt_, 8.8f, 0.1f, 1.0f); // Q
   SetMeasureNoiseCovatiance(6.0f, 1.0f, 0.01f); // R
   SetInitialCovariance(); // P0
   UKF::x_pre_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, oxts_data.pitch, oxts_data.roll, oxts_data.yaw, oxts_data.wy, 
                  oxts_data.wx, oxts_data.wz, UKF::gamma_a_, UKF::gamma_pitch_, UKF::gamma_roll_, UKF::gamma_yaw_, UKF::gamma_z_;
   UKF::p_pre_ = UKF::P0_;
}

// Convert GPS data Latitude/Longitude/Altitude -> meters, update Odometer (mx, my, mz).
void
 UKF::GPSConvertor(Odometer &odom,
                   const Oxts_Data &sensor_data_now, 
                   const Oxts_Data &sensor_data_pre){
   auto arc = 2.0f * UKF::PI_ * (UKF::earthRadius_ + sensor_data_now.alt)/360; 
   auto dx = arc * (float) cos(sensor_data_now.lat * UKF::PI_/180.0f) * (sensor_data_now.lon - sensor_data_pre.lon); // [m]
   auto dy = arc * (sensor_data_now.lat - sensor_data_pre.lat); // [m]
   auto dz = sensor_data_now.alt - sensor_data_pre.alt;
   odom.mx_ += dx;
   odom.my_ += dy;
   odom.mz_ += dz;
   odom.ds_ = sqrt(dx*dx + dy*dy + dz*dz); 
}

// Read sensor data and save to 'measurement_' matrix [7x1]
void 
 UKF::GetMeasurement(const Odometer &odom, const Oxts_Data oxts_data){
    UKF::measurements_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, oxts_data.wy, oxts_data.wx, oxts_data.wz;
}


// Eigen::MatrixXd 
//  UKF::GenerateSigmaPoints(const Eigen::VectorXd x, const Eigen::MatrixXd P, const double lambda, const int num_sig) {
//  }


// Initialize odometers to 0. 
void 
 Odometer::Initialize(){
    Odometer::mx_ = 0;
    Odometer::my_ = 0;
    Odometer::mz_ = 0;
    Odometer::ds_ = 0; 
}