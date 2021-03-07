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
#include "Function_Support.cpp"

const bool DISPLAY = true;

int main(int argc, char** argv){
    Filters<pcl::PointXYZ> filter;
    Features<pcl::PointXYZ> feature;
    Segmentation<pcl::PointXYZ> segmentation;
    Registration<pcl::PointXYZ> registration;
    Support<pcl::PointXYZ> sup;
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");

    /*------ Load files ------*/
    const std::string folderPath = "../../Test_data/data_2/"; // File path
    int16_t fileNum;
    std::vector<std::string> filePaths;
    std::tie(filePaths, fileNum) = sup.load_file(folderPath); // Load file path

    // Loop through all files
    int16_t NUM = 0;
    viewer.setBackgroundColor(0, 0, 0); // Set black background
    viewer.initCameraParameters();
    viewer.setCameraPosition(0, 0, 25, 1, 0, 1);
    while(NUM != fileNum){
        if(DISPLAY == true){ // Clear viewer
            viewer.removeAllPointClouds();
            viewer.removeAllShapes();
        }
        auto start_frame = std::chrono::system_clock::now();
        auto cloud = sup.load_pcd(filePaths, NUM);

        /*------ Down Sampling ------*/
        auto timer_down = std::chrono::system_clock::now(); // Start timer
        auto cloud_source_down = filter.VoxelGridDownSampling(cloud, 0.2f);
        sup.timer_calculate(timer_down, "Down Sampling");

        if(DISPLAY == true){
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_h(cloud_source_down, 180, 20, 20);
            viewer.addPointCloud(cloud_source_down, cloud_h, "cloud");
        }
        sup.timer_calculate(start_frame, "Per frame");
        NUM ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        viewer.spinOnce();
    }
    
    return 0;
}