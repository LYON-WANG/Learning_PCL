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
        float mx_;  
        float my_;  
        float mz_;  
        float ds_;

        void Initialize();
};

class UKF
{
public:
    // sensor sampling rate 10 Hz 
    float dt_ = 0.1;
    // State dimension (Augumented)
    int num_x_;
    // Augmented state dimension
    int num_aug_x_;
    // Sigma point spreading parameter
    double lambda_;
    // Process Noise Covariance Matrix Q
    Eigen::Matrix<float, 10, 10> Q_;
    // Measurement Noise Covariance Matrix R
    Eigen::Matrix<float, 7, 7> R_;
    // Initial Uncertainty Covariance P0
    Eigen::Matrix<float, 15, 15> P0_; // 15 States: [x,y,z, V, theta(pitch),psi(roll),phi(yaw), dot_theta,dot_psi,dot_phi, 
                                      //             gamma_a, gamma_theta, gamma_psi, gamma_phi, gamma_z]

    float gamma_a_ = 0;
    float gamma_pitch_ = 0;
    float gamma_roll_ = 0;
    float gamma_yaw_ = 0;
    float gamma_z_ = 0;

    Eigen::Matrix<float, 15, 1> x_pre_;
    Eigen::Matrix<float, 15, 15> p_pre_;

    Eigen::Matrix<float, 7, 1> measurements_;

    void SetProcessNoiseCovatiance(const float dt, const float sGPS = 8.8f, const float sCourse = 0.1f, const float sTurnRate = 1.0f);
    void SetMeasureNoiseCovatiance(const float var_GPS = 6.0f, const float var_speed = 1.0f, const float var_turn_angle = 0.01f);
    void SetInitialCovariance();
    void Initialize(const Odometer &odo, const Oxts_Data &oxts_data); // Initializa UKF (X_0, P_0)
    void GPSConvertor(Odometer &odo, const Oxts_Data &measurement_now, const Oxts_Data &measurement_pre);
    void GetMeasurement(const Odometer &odo, const Oxts_Data oxts_data);
    Eigen::MatrixXd GenerateSigmaPoints(Eigen::VectorXd x, Eigen::MatrixXd P, double const lambda, const int num_sig);

private:
    const double PI_= 3.14159;
    const int earthRadius_ = 6378388;

};


#endif /* UKF_H_ */