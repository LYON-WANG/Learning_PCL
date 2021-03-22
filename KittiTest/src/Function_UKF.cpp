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
 UKF::SetProcessNoiseCovatiance(const double &dt, const double &sGPS, const double &sCourse, 
                                 const double &sTurnRate){
   Eigen::MatrixXd i = Eigen::MatrixXd::Identity(10, 10);  
   Eigen::DiagonalMatrix<double, 10> m;
   m.diagonal() << 0.5 * sGPS * dt * dt, 0.5 * sGPS * dt * dt, 0.5 * sGPS * dt * dt, sGPS * dt, 
                    sCourse * dt, sCourse * dt, sCourse * dt, sTurnRate * dt, sTurnRate * dt, sTurnRate * dt;
   Q_ = i * m;
}

// Set measurement noise covariance R [7, 7]
void 
 UKF::SetMeasureNoiseCovatiance(const double &var_GPS, const double &var_speed, const double &var_turn_angle){
   Eigen::MatrixXd i = Eigen::MatrixXd::Identity(7, 7);  
   Eigen::DiagonalMatrix<double, 7> m;
   m.diagonal() << var_GPS * var_GPS, var_GPS * var_GPS, var_GPS * var_GPS, var_speed * var_speed,
                   var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle;
   R_ = i * m;
}

// Set initial uncertainty P0 [15, 15]
void 
 UKF::SetInitialCovariance(){
   Eigen::MatrixXd i = Eigen::MatrixXd::Identity(15, 15);  
   P0_ = i * 1000.0;
}

// Initialize state x_f_ = X0 and p_f_ = P0 
void 
 UKF::Initialize(const Odometer &odom, const Oxts_Data &oxts_data){
   // UKF::Q_ = Eigen::MatrixXd::Zero(10, 10);
   // UKF::R_ = Eigen::MatrixXd::Zero(7, 7);
   // UKF::P0_ = Eigen::MatrixXd::Zero(15, 15);
   // UKF::SP_ = Eigen::MatrixXd::Zero(15, 31);
   // UKF::W_ = Eigen::MatrixXd::Ones(1, 31);

   UKF::measurements_ = Eigen::MatrixXd::Zero(7, 1);

   // UKF Initialize noise covariance
   SetProcessNoiseCovatiance(UKF::dt_, 8.8, 0.1, 1.0); // Q
   SetMeasureNoiseCovatiance(6.0, 1.0, 0.01); // R
   SetInitialCovariance(); // P0
   UKF::x_f_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, oxts_data.pitch, oxts_data.roll, oxts_data.yaw, oxts_data.wy, 
                  oxts_data.wx, oxts_data.wz, UKF::gamma_a_, UKF::gamma_pitch_, UKF::gamma_roll_, UKF::gamma_yaw_, UKF::gamma_z_;
   UKF::p_f_ = UKF::P0_;
}

// Read sensor data and save to 'measurement_' matrix [7x1]
void 
 UKF::GetMeasurement(const Odometer &odom, const Oxts_Data &oxts_data){
    UKF::measurements_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, oxts_data.wy, oxts_data.wx, oxts_data.wz;
}

// Generate Sigma Points and sigma point Weights
void 
 UKF::GenerateSigmaPoints(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P){
   UKF::SP_.col(0) = x; 
   // UKF::lambda_ / (UKF::num_x_ + UKF::lambda_);
   // Calculate square root of P
   Eigen::MatrixXd C = P.llt().matrixL(); // Cholskey decompposition
   for(int i = 0; i < UKF::num_x_; i++){
      UKF::SP_.col(i + 1) = x + sqrt(UKF::lambda_ + UKF::num_x_) * C.col(i);
      UKF::SP_.col(i + 1 + UKF::num_x_) = x - sqrt(UKF::lambda_ + UKF::num_x_) * C.col(i);
   }
   UKF::W_ = Eigen::MatrixXd::Ones(1, 31) * (0.5 / (UKF::num_x_ + UKF::lambda_)); // Sigma point weights [W]
   UKF::W_(0) = UKF::lambda_ / (UKF::num_x_ + UKF::lambda_);
}

// Predict Sigma Points
void 
 UKF::PredictSigmaPoints(const Eigen::MatrixXd &SP, const Eigen::MatrixXd &W, const double &dt){
   UKF::x_p_.fill(0); // Predicted State Mean
   UKF::p_p_.fill(0); // Predicted State Covariance
   // CTRV motion model
   for(int i = 0; i < 2 * UKF::num_x_ + 1; i++){
      double p_x = SP(0, i); // pos x
      double p_y = SP(1, i); // pos y
      double p_z = SP(2, i); // pos z
      double   v = SP(3, i); // Speed
      double pitch = SP(4, i); // Pitch
      double roll  = SP(5, i); // Roll
      double yaw   = SP(6, i); // Yaw
      double pitch_rate = SP(7, i); // Pitch
      double roll_rate  = SP(8, i); // Roll
      double yaw_rate   = SP(9, i); // Yaw
      double gamma_acc  = SP(10, i);  // Gamma_Acceleration
      double gamma_pitch = SP(11, i); // Gamma_Pitch
      double gamma_roll  = SP(12, i); // Gamma_Roll
      double gamma_yaw   = SP(13, i); // Gamma_Yaw
      double gamma_z     = SP(14, i); // Gamma_Z
      if(yaw_rate > 1e-3){
         UKF::SP_predict_(0, i) = p_x + v/yaw_rate * (sin(yaw + yaw_rate * dt) - sin(yaw)) + 0.5 * dt * dt * cos(yaw) * gamma_acc;  // Position X
         UKF::SP_predict_(1, i) = p_y + v/yaw_rate * (-cos(yaw + yaw_rate * dt) + cos(yaw)) + 0.5 * dt * dt * sin(yaw) * gamma_acc; // Position Y
         UKF::SP_predict_(2, i) = p_z + dt * gamma_z;   // Position Z
         UKF::SP_predict_(3, i) = v + dt * gamma_acc;   // Speed V
         UKF::SP_predict_(4, i) = pitch + pitch_rate * dt + 0.5 * dt * dt * gamma_pitch; // Pitch
         UKF::SP_predict_(5, i) = roll + roll_rate * dt + 0.5 * dt * dt * gamma_roll;    // Roll
         UKF::SP_predict_(6, i) = yaw + yaw_rate * dt + 0.5 * dt * dt * gamma_yaw;       // Yaw
         UKF::SP_predict_(7, i) = pitch_rate + dt * gamma_pitch; // Pitch Rate
         UKF::SP_predict_(8, i) = roll_rate + dt * gamma_roll;   // Roll Rate
         UKF::SP_predict_(9, i) = yaw_rate + dt * gamma_yaw;     // Yaw Rate
      }
      else{
         UKF::SP_predict_(0, i) = p_x + v * cos(yaw) * dt + 0.5 * dt * dt * cos(yaw) * gamma_acc; // Position X 
         UKF::SP_predict_(1, i) = p_y + v * sin(yaw) * dt + 0.5 * dt * dt * sin(yaw) * gamma_acc; // Position Y
         UKF::SP_predict_(2, i) = p_z + dt * gamma_z; // Position Z
         UKF::SP_predict_(3, i) = v + dt * gamma_acc;   //Speed V
         UKF::SP_predict_(4, i) = pitch + pitch_rate * dt + 0.5 * dt * dt * gamma_pitch; // Pitch
         UKF::SP_predict_(5, i) = roll + roll_rate * dt + 0.5 * dt * dt * gamma_roll;    // Roll
         UKF::SP_predict_(6, i) = yaw + yaw_rate * dt + 0.5 * dt * dt * gamma_yaw;       // Yaw
         UKF::SP_predict_(7, i) = pitch_rate + dt * gamma_pitch; // Pitch Rate
         UKF::SP_predict_(8, i) = roll_rate + dt * gamma_roll;   // Roll Rate
         UKF::SP_predict_(9, i) = yaw_rate + dt * gamma_yaw;     // Yaw Rate
      }
      UKF::x_p_ = UKF::x_p_ + W(i) * UKF::SP_predict_.col(i); // Predicted State Mean
      
   }
   std::cout << UKF::x_p_ << std::endl;
}

// Predict state vector and covariance
void 
 UKF::Prediction(const Eigen::VectorXd &x, const Eigen::MatrixXd &P){
   GenerateSigmaPoints(x, P);
   PredictSigmaPoints(UKF::SP_, UKF::W_, UKF::dt_);
   
}

// Initialize odometers to 0. 
void 
 Odometer::Initialize(){
    Odometer::mx_ = 0.0;
    Odometer::my_ = 0.0;
    Odometer::mz_ = 0.0;
    Odometer::ds_ = 0.0; 
}

// Convert GPS data Latitude/Longitude/Altitude -> meters, update Odometer (mx, my, mz).
void
 Odometer::GPSConvertor(const Oxts_Data &sensor_data_now, const Oxts_Data &sensor_data_pre){
   double arc = 2.0 * Odometer::PI_ * (Odometer::earthRadius_ + sensor_data_now.alt)/360.0; 
   double dx = arc * (double) cos(sensor_data_now.lat * Odometer::PI_/180.0) * (sensor_data_now.lon - sensor_data_pre.lon); // [m]
   double dy = arc * (sensor_data_now.lat - sensor_data_pre.lat); // [m]
   double dz = sensor_data_now.alt - sensor_data_pre.alt;
   Odometer::mx_ += dx;
   Odometer::my_ += dy;
   Odometer::mz_ += dz;
   Odometer::ds_ = sqrt(dx*dx + dy*dy + dz*dz); 
}