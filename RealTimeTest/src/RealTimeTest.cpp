/* \author Leo Wang */
// Real-time object-detection, plane segmentation and registration test
// using PCL

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

    /*------ Load files ------*/
    const std::string folderPath = "../../Test_data/data_2/"; // File path
    int16_t fileNum;
    std::vector<std::string> filePaths;
    std::tie(filePaths, fileNum) = user.load_file(folderPath); // Load file path

    // Loop through all files
    int16_t NUM = 0;
    CameraAngle camera_angle = TOP; // Set camera angle
    user.initCamera(viewer, BLACK, camera_angle); // Initialize viewer
    while(NUM != fileNum){
        if(DISPLAY == true){ // Clear viewer
            viewer.removeAllPointClouds();
            //viewer.removeAllShapes();
        }
        auto start_frame = std::chrono::system_clock::now();// Start frame timer
        auto cloud = user.load_pcd(filePaths, NUM);

        /*------ 2. Down Sampling ------*/
        auto timer_downsampling = std::chrono::system_clock::now(); // Start down sampling timer
        auto cloud_source_down = filter.VoxelGridDownSampling(cloud, 0.2f); // PCLPointCloud2 --> pcl::PointXYZ
        user.timer_calculate(timer_downsampling, "Down Sampling"); // Print time

        /*------ 3. Pass Filter ------*/
        auto timer_passfilter = std::chrono::system_clock::now(); // Start pass filter timer
        auto cloud_source_pass = filter.PassThroughFilter(cloud_source_down, "z", std::array<float, 2> {-3.0f, -1.0f});
        std::sort(cloud_source_pass->points.begin(),cloud_source_pass->points.end(),point_cmp); // Resort points in Z axis
        user.timer_calculate(timer_passfilter, "Pass filter"); // Print time

        if(DISPLAY == true){
            user.showPointcloud(viewer, cloud_source_down, 2, RED, "PCD down");
        }
        user.timer_calculate(start_frame, "Per frame"); // Print frame timer
        std::cout << " " << std::endl; 
        NUM ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay for replaying
        viewer.spinOnce();
    }
    
    return 0;
}