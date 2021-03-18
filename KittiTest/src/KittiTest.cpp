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

const bool DISPLAY = true;

int main(int argc, char** argv){
    Filters<pcl::PointXYZ> filter;
    Features<pcl::PointXYZ> feature;
    Segmentation<pcl::PointXYZ> segmentation;
    Registration<pcl::PointXYZ> registration;
    User<pcl::PointXYZ> user;
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
    CameraAngle camera_angle = TOP; // Set camera angle
    user.initCamera(viewer, BLACK, camera_angle); // Initialize viewer
    while(NUM != fileNum){
        if(DISPLAY == true){ // Clear viewer
            viewer.removeAllPointClouds();
        }
        // Load KITTI -> PCD
        std::cout << "\nFrame [" << NUM << "]:" << std::endl;
        auto cloud = user.loadKitti(pcd_paths, NUM);
        // Load IMU & GPS
        auto oxts_data = user.loadOxts(oxts_paths, NUM);
        

        /*------ Visualization ------*/
        if(DISPLAY == true){
            user.showPointcloud(viewer, cloud, 2, WHITE, "PCD");
        }

        NUM ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay for replaying
        viewer.spinOnce();
    }

    return 0;
}