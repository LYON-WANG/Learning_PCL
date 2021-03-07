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
    std::tie(filePaths, fileNum) = user.loadFile(folderPath); // Load file path

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
        auto cloud = user.loadPCD(filePaths, NUM);

        /*------ 2. Down Sampling ------*/
        auto timer_downsampling = std::chrono::system_clock::now(); // Start down sampling timer
        auto cloud_source_down = filter.VoxelGridDownSampling(cloud, 0.3f); // PCLPointCloud2 --> pcl::PointXYZ
        user.timerCalculator(timer_downsampling, "Down Sampling"); // Print time

        /*------ 2. Statistical Outlier Removal ------*/
        auto timer_outlier = std::chrono::system_clock::now(); // Start timer
        cloud_source_down = filter.StatisticalOutlierRemoval(cloud_source_down, 30, 1.0);
        user.timerCalculator(timer_outlier, "Outlier Removal");

        /*------ 3. Pass Filter ------*/
        auto timer_passfilter = std::chrono::system_clock::now(); // Start pass filter timer
        auto cloud_source_pass = filter.PassThroughFilter(cloud_source_down, "z", std::array<float, 2> {-3.0f, -1.0f});
        user.timerCalculator(timer_passfilter, "Pass filter"); // Print time

        /*------ 4. Plane Segmentation ------*/
        // Rough ground segmentation
        auto timer_plane = std::chrono::system_clock::now(); // Start plane seg timer
        std::sort(cloud_source_down->points.begin(),cloud_source_down->points.end(),point_cmp); // Resort points in Z axis
        auto RoughGroundPoints = segmentation.RoughGroundExtraction(cloud_source_down, 1.2, 40);
        user.timerCalculator(timer_plane, "Rough Ground Extraction");
        // RANSAC Segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_road(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // Plane inliers
        auto timer_RAS = std::chrono::system_clock::now(); // Start RANSAC timer
        std::tie(cloud_road, cloud_other) = segmentation.PlaneSegmentation(cloud_source_down, RoughGroundPoints, 100, 0.2);
        user.timerCalculator(timer_RAS, "RANSAC Segmentation");
        user.timerCalculator(timer_plane, "Total plane Segmentation");


        /*------ Visualization ------*/
        if(DISPLAY == true){
            //user.showPointcloud(viewer, cloud_source_down, 2, RED, "original PCD");
            user.showPointcloud(viewer, cloud_road, 2, GREEN, "PCD road");
            user.showPointcloud(viewer, cloud_other, 2, RED, "PCD other");
        }
        user.timerCalculator(start_frame, "Per frame"); // Print frame timer
        std::cout << " " << std::endl; 
        NUM ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay for replaying
        viewer.spinOnce();
    }
    
    return 0;
}