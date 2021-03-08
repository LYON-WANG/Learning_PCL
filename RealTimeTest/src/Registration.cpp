// Real-timeregistration test using PCL
/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */
#include "RealTimeTest.h"
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
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous(new pcl::PointCloud<pcl::PointXYZ>); // Point cloud last frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_now(new pcl::PointCloud<pcl::PointXYZ>);      // Point cloud this frame (now)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_register(new pcl::PointCloud<pcl::PointXYZ>); // Registrated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);    // Final result 

    /*------ Load files ------*/
    const std::string folderPath = "../../Test_data/data_1/"; // File path
    int16_t fileNum;
    std::vector<std::string> filePaths;
    std::tie(filePaths, fileNum) = user.loadFile(folderPath); // Load file path
    // Loop through all files
    int16_t NUM = 0;
    CameraAngle camera_angle = TOP; // Set camera angle
    user.initCamera(viewer, BLACK, camera_angle); // Initialize viewer
    while(NUM != fileNum){
        if(DISPLAY == true){ // Clear viewer
            viewer.removeAllPointClouds();
        }
        std::cout << "Frame [" << NUM << "]:" << std::endl;
        auto start_frame = std::chrono::system_clock::now();// Start frame timer
        auto cloud = user.loadPCD(filePaths, NUM);

        /*------ 2. Down Sampling ------*/
        auto timer_downsampling = std::chrono::system_clock::now(); // Start down sampling timer
        auto cloud_source_down = filter.VoxelGridDownSampling(cloud, 0.3f); // PCLPointCloud2 --> pcl::PointXYZ
        user.timerCalculator(timer_downsampling, "Down Sampling"); // Print time

        // Distance Box
        const Eigen::Vector4f min_point(-40, -25, -3, 1);
        const Eigen::Vector4f max_point(40, 25, 4, 1);
        cloud_source_down = filter.boxFilter(cloud_source_down, min_point, max_point); // Remove roof outliers

        /*------ Crop Box Filter ------*/
        const Eigen::Vector4f roof_min(-1.5, -1.7, -1, 1);
        const Eigen::Vector4f roof_max(2.6, 1.7, -0.4, 1);
        auto timer_cropbox = std::chrono::system_clock::now(); // Start crop box timer
        cloud_source_down = filter.boxFilter(cloud_source_down, roof_min, roof_max, true); // Remove roof outliers
        user.timerCalculator(timer_cropbox, "Crop Box Filter"); // Print time

        /*------ REGISTRATION ------*/
        // First frame
        if(NUM == 0)
            *cloud_previous = *cloud_source_down;


        /*------ Visualization ------*/
        if(DISPLAY == true){
            //user.showPointcloud(viewer, RoughGroundPoints, 2, BLUE, "original PCD");
            user.showPointcloud(viewer, cloud_previous, 2, GREEN, "PCD TEST");
        }
        user.timerCalculator(start_frame, "Per frame"); // Print frame timer
        std::cout << " " << std::endl; 
        NUM ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay for replaying
        viewer.spinOnce();
    }
    return 0;
}