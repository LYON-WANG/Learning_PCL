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
    
    // // /*------ Save data to .CSV file ------*/
    // std::ofstream AccRecordingFile;
    // std::ofstream GyroRecordingFile;
    // std::ofstream GpsRecordingFile;
    // //std::ofstream UkfRecoringFile;
    // AccRecordingFile.trunc;   // Clear the CSV file
    // GyroRecordingFile.trunc; 
    // GpsRecordingFile.trunc;  
    // GpsRecordingFile.open("../SaveCSV/GPS.csv", std::ios::out);
    // AccRecordingFile.open("../SaveCSV/Acc.csv", std::ios::out);
    // GyroRecordingFile.open("../SaveCSV/Gyro.csv", std::ios::out);
    // //UkfRecoringFile.open("/opt/saveCSV/EKF.csv", std::ios::out);


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
        
        // // /*------ Save data to .CSV file ------*/
        // GpsRecordingFile << std::to_string(oxts_now.lat) << ',' << std::to_string(oxts_now.lon) << ',' << std::to_string(oxts_now.alt) << ',' 
        //     << std::to_string(oxts_now.vf) << ',' << std::to_string(oxts_now.yaw)  << std::endl;
        // //----- ACC -----//
        // AccRecordingFile << std::to_string(oxts_now.ax) << ',' << std::to_string(oxts_now.ay) << ',' << std::to_string(oxts_now.az) << std::endl;
        // //----- Gyro -----//
        // GyroRecordingFile << std::to_string(oxts_now.wx) << ',' << std::to_string(oxts_now.wy) << ',' << std::to_string(oxts_now.wz) << std::endl;


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
            //std::cout << ukf.W_ << std::endl;

            /*------ UKF Update ------*/
            //odom.GPSConvertor(oxts_now, oxts_pre);
            //ukf.GetMeasurement(odom, oxts_now);
            //std::cout << "Measurement: \n" << ukf.measurements_ << std::endl;
            

            //oxts_pre = oxts_now; // Update oxts_pre (sensor data)
        }



        /*------ Visualization ------*/
        if(DISPLAY == true){
            user.showPointcloud(viewer, cloud, 2, WHITE, "PCD");
        }


        NUM ++;
        user.timerCalculator(frame_timer, "Every Frame");
        //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay for replaying
        viewer.spinOnce();
    }
    // GpsRecordingFile.close();
    // AccRecordingFile.close();
    // GyroRecordingFile.close();

    return 0;
}