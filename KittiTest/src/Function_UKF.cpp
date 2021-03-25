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
   Eigen::MatrixXd i(Eigen::MatrixXd::Identity(10, 10));  
   Eigen::DiagonalMatrix<double, 10> m;
   m.diagonal() << 0.5 * sGPS * dt * dt, 0.5 * sGPS * dt * dt, 0.5 * sGPS * dt * dt, sGPS * dt, 
                    sCourse * dt, sCourse * dt, sCourse * dt, sTurnRate * dt, sTurnRate * dt, sTurnRate * dt;
   Q_ = i * m;
}

// Set measurement noise covariance R [8, 8]
void 
 UKF::SetMeasureNoiseCovatiance(const double &var_GPS, const double &var_speed, const double &var_course, const double &var_turn_angle){
   Eigen::MatrixXd i(Eigen::MatrixXd::Identity(8, 8));  
   Eigen::DiagonalMatrix<double, 8> m;
   m.diagonal() << var_GPS * var_GPS, var_GPS * var_GPS, var_GPS * var_GPS, var_speed * var_speed, var_course * var_course,
                   var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle, var_turn_angle * var_turn_angle;
   R_ = i * m;
}

// Set initial uncertainty P0 [15, 15]
void 
 UKF::SetInitialCovariance(){
   Eigen::MatrixXd i(Eigen::MatrixXd::Identity(15, 15));  
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

   measurements_ = Eigen::MatrixXd::Zero(8, 1);

   // UKF Initialize noise covariance
   SetProcessNoiseCovatiance(UKF::dt_, 8.8, 0.1, 1.0); // Q
   SetMeasureNoiseCovatiance(6.0, 1.0, 0.01); // R
   SetInitialCovariance(); // P0
   x_f_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, oxts_data.pitch, oxts_data.roll, oxts_data.yaw, oxts_data.wy, 
                  oxts_data.wx, oxts_data.wz, gamma_a_, gamma_pitch_, gamma_roll_, gamma_yaw_, gamma_z_;
   p_f_ = P0_;
}

// Read sensor data and save to 'measurement_' matrix [7x1]
void 
 UKF::GetMeasurement(const Odometer &odom, const Oxts_Data &oxts_data){
   measurements_ << odom.mx_, odom.my_, odom.mz_, oxts_data.vf, oxts_data.yaw, oxts_data.wy, oxts_data.wx, oxts_data.wz;
}

// Generate Sigma Points and sigma point Weights
void 
 UKF::GenerateSigmaPoints(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P){
   SP_.col(0) = x; 
   // Calculate square root of P
   Eigen::MatrixXd C = P.llt().matrixL(); // Cholskey decompposition
   for(int i = 0; i < num_x_; i++){
      SP_.col(i + 1) = x + sqrt(lambda_ + num_x_) * C.col(i);
      SP_.col(i + 1 + num_x_) = x - sqrt(lambda_ + num_x_) * C.col(i);
   }
   W_ = Eigen::MatrixXd::Ones(1, 31) * (0.5 / (num_x_ + lambda_)); // Sigma point weights [W]
   W_(0) = lambda_ / (num_x_ + lambda_);
}

// Predict Sigma Points
void 
 UKF::PredictSigmaPoints(const Eigen::MatrixXd &SP, const Eigen::MatrixXd &W, const double &dt){
   UKF::x_p_.fill(0); // Predicted State Mean
   UKF::p_p_.fill(0); // Predicted State Covariance
   for(int i = 0; i < 2 * UKF::num_x_ + 1; i++){
      double p_x(SP(0, i)); // pos x
      double p_y(SP(1, i)); // pos y
      double p_z(SP(2, i)); // pos z
      double v(SP(3, i));   // Speed
      double pitch(SP(4, i));  // Pitch
      double roll(SP(5, i));   // Roll
      double yaw(SP(6, i));    // Yaw
      double pitch_rate(SP(7, i)); // Pitch
      double roll_rate(SP(8, i));  // Roll
      double yaw_rate(SP(9, i));   // Yaw
      double gamma_acc(SP(10, i));   // Gamma_Acceleration
      double gamma_pitch(SP(11, i)); // Gamma_Pitch
      double gamma_roll(SP(12, i));  // Gamma_Roll
      double gamma_yaw(SP(13, i));   // Gamma_Yaw
      double gamma_z(SP(14, i));     // Gamma_Z
      // CTRV motion model
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
      // CV model
      else{
         UKF::SP_predict_(0, i) = p_x + v * cos(yaw) * dt + 0.5 * dt * dt * cos(yaw) * gamma_acc; // Position X 
         UKF::SP_predict_(1, i) = p_y + v * sin(yaw) * dt + 0.5 * dt * dt * sin(yaw) * gamma_acc; // Position Y
         UKF::SP_predict_(2, i) = p_z + dt * gamma_z;   // Position Z
         UKF::SP_predict_(3, i) = v + dt * gamma_acc;   //Speed V
         UKF::SP_predict_(4, i) = pitch + pitch_rate * dt + 0.5 * dt * dt * gamma_pitch; // Pitch
         UKF::SP_predict_(5, i) = roll + roll_rate * dt + 0.5 * dt * dt * gamma_roll;    // Roll
         UKF::SP_predict_(6, i) = yaw + yaw_rate * dt + 0.5 * dt * dt * gamma_yaw;       // Yaw
         UKF::SP_predict_(7, i) = pitch_rate + dt * gamma_pitch; // Pitch Rate
         UKF::SP_predict_(8, i) = roll_rate + dt * gamma_roll;   // Roll Rate
         UKF::SP_predict_(9, i) = yaw_rate + dt * gamma_yaw;     // Yaw Rate
      }
      UKF::x_p_ += W(i) * UKF::SP_predict_.col(i); // Predicted State Mean
      // State Difference
      Eigen::Matrix<double, 10, 1> x_diff(UKF::SP_predict_.col(i) - UKF::x_p_);
      // Angle Normalization
      if(x_diff(6) > M_PI) 
         x_diff(6) -= 2.*M_PI;
      if(x_diff(6) < -M_PI)
         x_diff(6) += 2.*M_PI;
      UKF::p_p_ += W(i) * x_diff * x_diff.transpose();
   }
   
}

// Predict state vector and covariance
void 
 UKF::Prediction(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P){
   GenerateSigmaPoints(x, P);
   PredictSigmaPoints(UKF::SP_, UKF::W_, UKF::dt_);
}

void 
 UKF::Update(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P, 
             const Eigen::MatrixXd &measure, const Odometer &odom){
   // Generate Sigma Points 
   int lambda = 3 - P.rows();
   UKF::SP_U_.col(0) = x; 
   // Calculate square root of P
   Eigen::MatrixXd C = P.llt().matrixL(); // Cholskey decompposition
   for(int i = 0; i < x.rows(); i++){
      UKF::SP_U_.col(i + 1) = x + sqrt(lambda + x.rows()) * C.col(i);
      UKF::SP_U_.col(i + 1 + x.rows()) = x - sqrt(lambda + x.rows()) * C.col(i);
   }
   UKF::W_U_ = Eigen::MatrixXd::Ones(1, 21) * (0.5 / (x.rows() + lambda)); // Sigma point weights [W]
   UKF::W_U_(0) = lambda / (x.rows() + lambda);

   // Only IMU
   if(odom.ds_ == 0){
      Eigen::MatrixXd y_hat(Eigen::MatrixXd::Zero(3,1));
      for(int i = 0; i < W_U_.cols(); i++){
         y_hat(0) += SP_U_(7, i) * W_U_(i);
         y_hat(1) += SP_U_(8, i) * W_U_(i);
         y_hat(2) += SP_U_(9, i) * W_U_(i);
      }
      Eigen::Matrix<double, 10, 3> P_xy(Eigen::MatrixXd::Zero(10, 3));
      Eigen::Matrix<double, 3, 3> S;
      S << R_(4, 4), 0, 0, 0, R_(5, 5) , 0, 0, 0 , R_(6, 6);
      Eigen::Matrix<double, 3, 21> hSP;
      hSP << SP_U_.row(7), SP_U_.row(8), SP_U_.row(9);
      for(int i = 0; i < W_U_.cols(); i++){
         P_xy += W_U_(i) * (SP_U_.col(i) - x) * (hSP.col(i) - y_hat).transpose();
         S += W_U_(i) * (hSP.col(i) - y_hat) * (hSP.col(i) - y_hat).transpose();
      }
      // Calculate updated mean and covariance
      Eigen::Matrix<double, 3, 1> y;
      y << measure(5), measure(6), measure(7); // Pitch rate, Roll rate, Yaw rate
      // x_f_ = x + P_xy * S.inverse() * (y - y_hat);
      // p_f_ = P - P_xy * S.inverse() * P_xy.inverse();
   }
   // Both GPS & IMU
   else{
      Eigen::MatrixXd y_hat(Eigen::MatrixXd::Zero(8,1));
      for(int i = 0; i < UKF::W_U_.cols(); i++){
         y_hat(0) += UKF::SP_U_(0, i) * UKF::W_U_(i); // Position X
         y_hat(1) += UKF::SP_U_(1, i) * UKF::W_U_(i); // Position Y
         y_hat(2) += UKF::SP_U_(2, i) * UKF::W_U_(i); // Position Z
         y_hat(3) += UKF::SP_U_(3, i) * UKF::W_U_(i); // Speed V
         y_hat(4) += UKF::SP_U_(6, i) * UKF::W_U_(i); // Yaw
         y_hat(5) += UKF::SP_U_(7, i) * UKF::W_U_(i); // Pitch Rate
         y_hat(6) += UKF::SP_U_(8, i) * UKF::W_U_(i); // Roll Rate
         y_hat(7) += UKF::SP_U_(9, i) * UKF::W_U_(i); // Yaw Rate
      }
      Eigen::Matrix<double, 10, 8> P_xy(Eigen::MatrixXd::Zero(10, 8));
      Eigen::Matrix<double, 8, 8> S(R_);
      Eigen::Matrix<double, 8, 21> hSP;
      hSP << SP_U_.row(0), SP_U_.row(1),SP_U_.row(2), SP_U_.row(3), SP_U_.row(6), SP_U_.row(7), SP_U_.row(8), SP_U_.row(9);
      for(int i = 0; i < W_U_.cols(); i++){
         P_xy += W_U_(i) * (SP_U_.col(i) - x) * (hSP.col(i) - y_hat).transpose();
         S += W_U_(i) * (hSP.col(i) - y_hat) * (hSP.col(i) - y_hat).transpose();
      }
      // Calculate updated mean and covariance
      Eigen::Matrix<double, 8, 1> y(measurements_); // GPS + IMU
   }
   
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