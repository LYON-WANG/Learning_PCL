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

const float SENSOR_HEIGHT = 2;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NDT(new pcl::PointCloud<pcl::PointXYZ>); // NDT Registrated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP(new pcl::PointCloud<pcl::PointXYZ>); // ICP Registrated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>); // ICP Registrated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);    // Final result 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);    // Final result 
    Eigen::Matrix4f initial_guess_transMatrix = Eigen::Matrix4f::Identity (); // NDT initial guess
    Eigen::Matrix4f NDT_transMatrix = Eigen::Matrix4f::Identity (); // NDT transformation
    Eigen::Matrix4f ICP_transMatrix = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f global_transMatrix = Eigen::Matrix4f::Identity ();

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
        }
        std::cout << "\nFrame [" << NUM << "]:" << std::endl;
        auto start_frame = std::chrono::system_clock::now();// Start frame timer
        auto cloud = user.loadPCD(filePaths, NUM);

        /*------ 2. Down Sampling ------*/
        auto timer_downsampling = std::chrono::system_clock::now(); // Start down sampling timer
        auto cloud_down = filter.VoxelGridDownSampling(cloud, 0.3f); // PCLPointCloud2 --> pcl::PointXYZ
        user.timerCalculator(timer_downsampling, "Down Sampling"); // Print time

        // Distance Box
        const Eigen::Vector4f min_point(-40, -25, -3, 1);
        const Eigen::Vector4f max_point(40, 25, 4, 1);
        cloud_down = filter.boxFilter(cloud_down, min_point, max_point); // Remove roof outliers

        /*------ Crop Box Filter ------*/
        const Eigen::Vector4f roof_min(-1.5, -1.7, -1, 1);
        const Eigen::Vector4f roof_max(2.6, 1.7, -0.4, 1);
        auto timer_cropbox = std::chrono::system_clock::now(); // Start crop box timer
        cloud_down = filter.boxFilter(cloud_down, roof_min, roof_max, true); // Remove roof outliers
        user.timerCalculator(timer_cropbox, "Crop Box Filter"); // Print time

        /*------ 3. Plane Segmentation ------*/
        // Rough ground segmentation
        auto timer_plane = std::chrono::system_clock::now(); // Start plane seg timer
        std::sort(cloud_down->points.begin(),cloud_down->points.end(),point_cmp); // Resort points in Z axis
        cloud_down = filter.PassThroughFilter(cloud_down, "z", std::array<float, 2> {-SENSOR_HEIGHT-0.2, 1.0f}); // 'Z' Pass filter
        auto RoughGroundPoints = segmentation.RoughGroundExtraction(cloud_down, 1.0, 60);
        user.timerCalculator(timer_plane, "Rough Ground Extraction");
        // RANSAC Segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_road(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // Plane inliers
        auto timer_RAS = std::chrono::system_clock::now(); // Start RANSAC timer
        std::tie(cloud_road, cloud_other) = segmentation.PlaneSegmentationRANSAC(cloud_down, RoughGroundPoints, 150, 0.3);
        user.timerCalculator(timer_RAS, "RANSAC Segmentation");
        user.timerCalculator(timer_plane, "Total plane Segmentation");

        /*------ REGISTRATION ------*/
        // First frame
        if(NUM == 0){
            *cloud_previous = *cloud_other;
            *cloud_final += *cloud_previous;
        }
        else if(NUM%5 == 0 && NUM != 0){
            viewer.removeAllPointClouds();
            std::cout << "------------------------------- Registration [Frame " << NUM - 2 << ", " << NUM<< "] -----------------------------------------" << std::endl; 
            *cloud_now = *cloud_other;
            
            // NDT registration
            auto timer_NDT = std::chrono::system_clock::now(); // Start NDT timer
            std::tie(cloud_NDT, NDT_transMatrix) = registration.NDT_Registration(cloud_previous, cloud_now, initial_guess_transMatrix, 1e-2, 0.2, 2.0, 10);
            user.timerCalculator(timer_NDT, "NDT registration"); // Print time

            // Estimate normals
            std::cout << "\n...Estimate normals..." << std::endl;
            auto timer_normal = std::chrono::system_clock::now(); // Start timer
            auto cloud_NDT_normal = feature.Normal_Estimation(cloud_NDT, 30);
            auto cloud_now_normal = feature.Normal_Estimation(cloud_now, 30);
            user.timerCalculator(timer_normal, "Normal Estimation");
            
            // ICP registration
            auto timer_ICP = std::chrono::system_clock::now(); // Start ICP timer
            std::tie(cloud_ICP, ICP_transMatrix) = registration.ICP_Point2Point(cloud_NDT, cloud_now, NDT_transMatrix, 100, 1e-6, 0.2);
            //std::tie(cloud_ICP, ICP_transMatrix) = registration.ICP_Point2Plane(cloud_NDT_normal, cloud_now_normal, NDT_transMatrix, 100, 1e-6, 0.2);
            pcl::transformPointCloud (*cloud_now, *cloud_output, ICP_transMatrix.inverse()*NDT_transMatrix.inverse());
            user.timerCalculator(timer_ICP, "ICP registration"); // Print time

            // Transfer aligned cloud into global coordinate
            pcl::transformPointCloud (*cloud_output, *cloud_result, global_transMatrix);
            global_transMatrix = global_transMatrix * ICP_transMatrix.inverse()*NDT_transMatrix.inverse();
            std::cout << "Global Transform Matrix:\n" << global_transMatrix << std::endl;

            //Stitch aligned clouds
            *cloud_final += *cloud_result;
            *cloud_previous = *cloud_now;

        }
        /*------ Visualization ------*/
        if(DISPLAY == true){
            user.showPointcloud(viewer, cloud_now, 2, RED, "PCD");
            user.showPointcloud(viewer, cloud_ICP, 2, GREEN, "PCD TEST");
        }
        user.timerCalculator(start_frame, "Per frame"); // Print frame timer
        NUM ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Delay for replaying
        viewer.spinOnce();
    }
    pcl::io::savePCDFile("./../result.pcd", *cloud_final);
    std::cout << "Save registration result to 'result.pcd'. " << std::endl;
    std::cout << "Final cloud size: " << cloud_final->points.size() << std::endl;
    return 0;
}