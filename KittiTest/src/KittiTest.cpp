// KITTI dataset test
/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */
#include "main.h"
#include "Function_Filtering.cpp"
#include "Function_Features.cpp"
#include "Function_Segmentation.cpp"
#include "Function_Registration.cpp"
#include "Function_User.cpp"
#include "Function_UKF.cpp"

const bool DISPLAY = true;

int main(int argc, char** argv){
    Filters<pcl::PointXYZ> filter;
    Features<pcl::PointXYZ> feature;
    Segmentation<pcl::PointXYZ> segmentation;
    Registration<pcl::PointXYZ> registration;
    User<pcl::PointXYZ> user;
    UKF ukf;
    Odometer odom;
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");

    /*------ Load files ------*/
    const std::string pcd_path = "../../Test_data/2011_09_26/velodyne_points/data/"; // Point cloud data path
    const std::string oxts_path = "../../Test_data/2011_09_26/oxts/data/"; // IMU&GPS data path
    int16_t fileNum;
    std::vector<std::string> pcd_paths;
    std::vector<std::string> oxts_paths;
    std::tie(pcd_paths, fileNum) = user.loadFile(pcd_path); // Load file path
    std::tie(oxts_paths, fileNum) = user.loadFile(oxts_path); // Load file path
    
    // Loop through all files
    int16_t NUM = 0;
    Oxts_Data oxts_now;
    Oxts_Data oxts_pre;
    CameraAngle camera_angle = TOP; // Set camera angle
    user.initCamera(viewer, BLACK, camera_angle); // Initialize viewer
    while(NUM != 3){
        auto frame_timer = std::chrono::system_clock::now();
        if(DISPLAY == true){ // Clear viewer
            viewer.removeAllPointClouds();
        }
        // Load KITTI -> PCD
        std::cout << "\nFrame [" << NUM << "]:" << std::endl;
        auto cloud = user.loadKitti(pcd_paths, NUM);
        // Load IMU & GPS data
        oxts_now = user.loadOxts(oxts_paths, NUM);
       
        // Initialize UKF
        if(NUM < 1){
            odom.Initialize();
            ukf.Initialize(odom, oxts_now);
            oxts_pre = oxts_now; // Initialize oxts_pre (sensor data)
            ukf.GetMeasurement(odom, oxts_now);
            //std::cout << "Measurement: \n" << ukf.measurements_ << std::endl;
        }
        else{
            /*------ UKF Prediction ------*/
            ukf.Prediction(ukf.x_f_, ukf.p_f_);

            /*------ UKF Update ------*/
            odom.GPSConvertor(oxts_now, oxts_pre);
            ukf.GetMeasurement(odom, oxts_now);
            //std::cout << "Measurement: \n" << ukf.measurements_ << std::endl;
            ukf.Update(ukf.x_p_, ukf.p_p_, ukf.measurements_, odom);
            

            oxts_pre = oxts_now; // Update oxts_pre (sensor data)
        }

        /*------ Visualization ------*/
        if(DISPLAY == true){
            user.showPointcloud(viewer, cloud, 2, WHITE, "PCD");
        }


        NUM ++;
        user.timerCalculator(frame_timer, "Every Frame");
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay for replaying
        viewer.spinOnce();
    }

    return 0;
}