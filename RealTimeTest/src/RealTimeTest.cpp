// Real-time object-detection, plane segmentation and registration test using PCL
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
            viewer.removeAllShapes();
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

        /*------ 2. Statistical Outlier Removal ------*/
        // auto timer_outlier = std::chrono::system_clock::now(); // Start timer
        // cloud_source_down = filter.StatisticalOutlierRemoval(cloud_source_down, 30, 2.0);
        // user.timerCalculator(timer_outlier, "Outlier Removal");   // Took around 80-90 ms

        /*------ 3. Plane Segmentation ------*/
        // Rough ground segmentation
        auto timer_plane = std::chrono::system_clock::now(); // Start plane seg timer
        std::sort(cloud_source_down->points.begin(),cloud_source_down->points.end(),point_cmp); // Resort points in Z axis
        cloud_source_down = filter.PassThroughFilter(cloud_source_down, "z", std::array<float, 2> {-SENSOR_HEIGHT-0.2, 1.0f}); // 'Z' Pass filter
        auto RoughGroundPoints = segmentation.RoughGroundExtraction(cloud_source_down, 1.0, 60);
        user.timerCalculator(timer_plane, "Rough Ground Extraction");
        // RANSAC Segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_road(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // Plane inliers
        auto timer_RAS = std::chrono::system_clock::now(); // Start RANSAC timer
        std::tie(cloud_road, cloud_other) = segmentation.PlaneSegmentationRANSAC(cloud_source_down, RoughGroundPoints, 150, 0.3);
        user.timerCalculator(timer_RAS, "RANSAC Segmentation");
        user.timerCalculator(timer_plane, "Total plane Segmentation");

        /*------ Statistical Outlier Removal ------*/
        auto timer_outlier = std::chrono::system_clock::now(); // Start timer
        cloud_other = filter.StatisticalOutlierRemoval(cloud_other, 30, 2.0);
        user.timerCalculator(timer_outlier, "Outlier Removal");  // Around 30 ms

        /*------ 4. Object Detection ------*/
        auto timer_clustering = std::chrono::system_clock::now(); // Start RANSAC timer
        auto cloud_clusters = segmentation.EuclideanClustering(cloud_other, 0.5, 20, 300);
        int cluster_ID = 1;
        for(const auto &cluster : cloud_clusters){
            Box box = segmentation.findBoundingBox(cluster);
            if(DISPLAY == true)
                user.drawBoundingBox(viewer, box, cluster_ID, BLUE, 0.8);
            cluster_ID++;
        }
        user.timerCalculator(timer_clustering, "Euclidean Clustering");

        /*------ 5. Registration ------*/

        /*------ Visualization ------*/
        if(DISPLAY == true){
            //user.showPointcloud(viewer, RoughGroundPoints, 2, BLUE, "original PCD");
            user.showPointcloud(viewer, cloud_road, 2, GREEN, "PCD road");
            user.showPointcloud(viewer, cloud_other, 2, RED, "PCD other");
        }
        user.timerCalculator(start_frame, "Per frame"); // Print frame timer
        std::cout << " " << std::endl; 
        NUM ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Delay for replaying
        viewer.spinOnce();
    }
    return 0;
}