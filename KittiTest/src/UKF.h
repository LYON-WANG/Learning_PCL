/* \author Leo Wang & Varun Hegde*/
// Customized Supporting function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */
#ifndef UKF_H_
#define UKF_H_

#include "main.h"

class Odometer
{
    public:
        Odometer(){};
        double mx_;  
        double my_;  
        double mz_;  
        double ds_;

        void Initialize();
        void GPSConvertor(const Oxts_Data &measurement_now, const Oxts_Data &measurement_pre);

    private:
        const double PI_= 3.14159;
        const int earthRadius_ = 6378388;

};

class UKF
{
private:
    // sensor sampling rate 10 Hz 
    const double dt_ = 0.1;
    // State dimension (Augumented)
    const int num_x_ = 15;
public:
    // Augmented state dimension
    //const int num_aug_x_;
    // Sigma point spreading parameter
    double lambda_ = 3 - num_x_;
    // Process Noise Covariance Matrix Q
    Eigen::Matrix<double, 10, 10> Q_;
    // Measurement Noise Covariance Matrix R
    Eigen::Matrix<double, 7, 7> R_;
    // Initial Uncertainty Covariance P0
    Eigen::Matrix<double, 15, 15> P0_; // 15 States: [x,y,z, V, theta(pitch),psi(roll),phi(yaw), dot_theta,dot_psi,dot_phi, 
                                      //             gamma_a, gamma_theta, gamma_psi, gamma_phi, gamma_z]
    const double gamma_a_ = 0.0;
    const double gamma_pitch_ = 0.0;
    const double gamma_roll_ = 0.0;
    const double gamma_yaw_ = 0.0;
    const double gamma_z_ = 0.0;

    Eigen::Matrix<double, 15, 1> x_f_;  // Filtered Estimates
    Eigen::Matrix<double, 15, 15> p_f_; // Filtered Error Covariance
    Eigen::Matrix<double, 10, 1> x_p_;  // Predicted Estimates
    Eigen::Matrix<double, 15, 15> p_p_; // Predicted Error Covariance

    // Sigma points and weights
    Eigen::Matrix<double, 15, 31> SP_; // Sigma points
    Eigen::Matrix<double, 1, 31> W_;   // Sigma point Weights
    Eigen::Matrix<double, 10, 31> SP_predict_; // Sigma points

    // Measurement matrix
    Eigen::Matrix<double, 7, 1> measurements_;

    void SetProcessNoiseCovatiance(const double &dt, const double &sGPS = 8.8, const double &sCourse = 0.1, const double &sTurnRate = 1.0);
    void SetMeasureNoiseCovatiance(const double &var_GPS = 6.0, const double &var_speed = 1.0, const double &var_turn_angle = 0.01);
    void SetInitialCovariance();
    void Initialize(const Odometer &odo, const Oxts_Data &oxts_data); // Initializa UKF (X_0, P_0)
    void GetMeasurement(const Odometer &odo, const Oxts_Data &oxts_data);
    void GenerateSigmaPoints(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P);
    void PredictSigmaPoints(const Eigen::MatrixXd &SP, const Eigen::MatrixXd &W, const double &dt);
    void Prediction(const Eigen::VectorXd &x, const Eigen::MatrixXd &P);
};


#endif /* UKF_H_ */