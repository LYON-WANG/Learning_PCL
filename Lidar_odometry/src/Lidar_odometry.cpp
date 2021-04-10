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
const float SENSOR_HEIGHT = 2;

int main(int argc, char** argv){
    Filters<pcl::PointXYZ> filter;
    Features<pcl::PointXYZ> feature;
    Segmentation<pcl::PointXYZ> segmentation;
    Registration<pcl::PointXYZ> registration;
    User<pcl::PointXYZ> user;
    UKF ukf;
    Odometer odom;
    pcl::visualization::PCLVisualizer viewer("PCD Viewer");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_previous(new pcl::PointCloud<pcl::PointXYZ>); // Point cloud last frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_now(new pcl::PointCloud<pcl::PointXYZ>);      // Point cloud this frame (now)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NDT(new pcl::PointCloud<pcl::PointXYZ>); // NDT Registrated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP(new pcl::PointCloud<pcl::PointXYZ>); // ICP Registrated point cloud
    //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ICP(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>); // ICP Registrated point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);    // Final result 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);    // Final result 
    Eigen::Matrix4f initial_guess_transMatrix = Eigen::Matrix4f::Identity (); // NDT initial guess
    Eigen::Matrix4f NDT_transMatrix = Eigen::Matrix4f::Identity (); // NDT transformation
    Eigen::Matrix4f ICP_transMatrix = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f global_transMatrix = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f GPS_IMU_to_Velodyne_kitti = Eigen::Matrix4f::Identity (); // 

    /*------ Load files ------*/
    const std::string pcd_path = "../../Test_data/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/"; // Point cloud data path
    const std::string oxts_path = "../../Test_data/2011_09_26/2011_09_26_drive_0005_sync/oxts/data/"; // IMU&GPS data path
    int16_t fileNum;
    std::vector<std::string> pcd_paths;
    std::vector<std::string> oxts_paths;
    std::tie(pcd_paths, fileNum) = user.loadFile(pcd_path); // Load file path
    std::tie(oxts_paths, fileNum) = user.loadFile(oxts_path); // Load file path

    
    // Loop through all files
    int16_t NUM = 0;
    Oxts_Data oxts_now;
    Oxts_Data oxts_pre;
    std::vector<double> GPSX, GPSY, LidarodoX, LidarodoY;// For matplotlib
    CameraAngle camera_angle = TOP; // Set camera angle
    user.initCamera(viewer, BLACK, camera_angle); // Initialize viewer
    while(NUM != fileNum){
        auto frame_timer = std::chrono::system_clock::now();
        if(DISPLAY == true){ // Clear viewer
            viewer.removeAllPointClouds();
        }
        // Load KITTI -> PCD  
        std::cout << "\nFrame [" << NUM << "]:" << std::endl;
        auto cloud = user.loadKitti(pcd_paths, NUM);
        // Load IMU & GPS data
        oxts_now = user.loadOxts(oxts_paths, NUM);

        auto timer_downsampling = std::chrono::system_clock::now(); // Start down sampling timer
        //auto cloud_source_down = filter.VoxelGridDownSampling(cloud, 0.3f); // PCLPointCloud2 --> pcl::PointXYZ
        float down_samp = 0.3f;
        pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
        voxelFilter.setInputCloud(cloud);
        voxelFilter.setLeafSize(down_samp, down_samp, down_samp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_down(new pcl::PointCloud<pcl::PointXYZ>); // Point cloud downsampled
        voxelFilter.filter(*cloud_source_down);
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

        /*------ 3. Plane Segmentation ------*/
        // Rough ground segmentation
        auto timer_plane = std::chrono::system_clock::now(); // Start plane seg timer
        //std::sort(cloud_source_down->points.begin(),cloud_source_down->points.end(),point_cmp); // Resort points in Z axis
        cloud_source_down = filter.PassThroughFilter(cloud_source_down, "z", std::array<float, 2> {-SENSOR_HEIGHT-0.2, 1.0f}); // 'Z' Pass filter
        auto RoughGroundPoints = segmentation.RoughGroundExtraction(cloud_source_down, 1.0, 60);
        user.timerCalculator(timer_plane, "Rough Ground Extraction");


        // RANSAC Segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_road(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // Plane inliers

        auto timer_RAS = std::chrono::system_clock::now(); // Start RANSAC timer
        std::tie(cloud_road, cloud_other) = segmentation.PlaneSegmentationRANSAC(cloud_source_down, RoughGroundPoints, 150, 0.3);
        user.timerCalculator(timer_RAS, "RANSAC Segmentation");
        user.timerCalculator(timer_plane, "Total plane Segmentation");

        int state_dim= 6;

        std::vector<double> States_from_trans (state_dim,0);

        /*------ REGISTRATION ------*/
        // First frame
        if(NUM == 0){
            std::cout << "enterloop1"<< std::endl;
            *cloud_previous = *cloud_other;
            std::cout << "enterloop2"<< std::endl;
            *cloud_final += *cloud_previous;
            std::cout << "enterloop3"<< std::endl;
            odom.Initialize(); // Initialize Odometer
            ukf.Initialize(odom, oxts_now); // Initialize UKF Q, R, P0, x_f, p_f
            oxts_pre = oxts_now; // Initialize oxts_pre (sensor data) for future odometer calculation
            ukf.GetMeasurement(odom, oxts_now); // Get first measurement (Actually not used, only for plotting.)
        }
        //else if(NUM%5 == 0 && NUM != 0){
        else if(NUM != 0){    
            viewer.removeAllPointClouds();
            std::cout << "enterloop"<< std::endl;
            //std::cout << "------------------------------- Registration [Frame " << NUM - 5 << ", " << NUM<< "] -----------------------------------------" << std::endl; 
            *cloud_now = *cloud_other;
            
            // NDT registration
            auto timer_NDT = std::chrono::system_clock::now(); // Start NDT timer
            std::tie(cloud_NDT, NDT_transMatrix) = registration.NDT_Registration(cloud_previous, cloud_now, initial_guess_transMatrix, 1e-2, 0.2, 3.0, 10);
            user.timerCalculator(timer_NDT, "NDT registration"); // Print time

            // ICP registration
            auto timer_ICP = std::chrono::system_clock::now(); // Start ICP timer
            std::tie(cloud_ICP, ICP_transMatrix) = registration.ICP_Point2Point(cloud_NDT, cloud_now, NDT_transMatrix, 100, 1e-7, 0.6);
            //std::tie(cloud_ICP, ICP_transMatrix) = registration.ICP_Point2Plane(cloud_NDT_normal, cloud_now_normal, NDT_transMatrix, 100, 1e-6, 0.2);
            pcl::transformPointCloud (*cloud_now, *cloud_output, ICP_transMatrix.inverse() * NDT_transMatrix.inverse());
            user.timerCalculator(timer_ICP, "ICP registration"); // Print time
            
            // Transfer aligned cloud into global coordinate
            pcl::transformPointCloud (*cloud_output, *cloud_result, global_transMatrix);
            global_transMatrix = global_transMatrix * ICP_transMatrix.inverse()*NDT_transMatrix.inverse();
            std::cout << "Global Transform Matrix:\n" << global_transMatrix << std::endl;
            
            // states order x,y,z,pitch,roll,yaw
            std::vector<double> States_from_trans = user.Transformmatrix_to_states(global_transMatrix);
        

            //Stitch aligned clouds
            *cloud_final += *cloud_result;
            *cloud_previous = *cloud_now;

            /*------ UKF Update ------*/
            odom.GPSConvertor(oxts_now, oxts_pre); // Convert GPS coordinates to mileage [meter]
            ukf.GetMeasurement(odom, oxts_now); // Get measurement for update step
            //ukf.Update(ukf.x_p_, ukf.p_p_, ukf.measurements_, odom); 
            oxts_pre = oxts_now; // Update oxts_pre (sensor data)

            if(DISPLAY == true){
            user.showPointcloud(viewer, cloud_final, 2, WHITE, "PCD");
            // matplot
            LidarodoX.push_back(States_from_trans[1]);
            LidarodoY.push_back(-States_from_trans[0]);
            GPSX.push_back(ukf.measurements_(0,0));
            GPSY.push_back(ukf.measurements_(1,0));
            ukf.Plot(GPSX, GPSY, LidarodoX, LidarodoY);
        }
        std::cout << " X axis absolute difference a bsolute in meters" << abs(LidarodoX.back()-GPSX.back());
        std::cout << " Y axis absolute difference absolute in meters" << abs(LidarodoY.back()-GPSY.back());


        }
        /*---------------          UKF          ---------------*/
        // Initialize UKF
/*
        if(NUM < 1){
            odom.Initialize(); // Initialize Odometer
            ukf.Initialize(odom, oxts_now); // Initialize UKF Q, R, P0, x_f, p_f
            oxts_pre = oxts_now; // Initialize oxts_pre (sensor data) for future odometer calculation
            ukf.GetMeasurement(odom, oxts_now); // Get first measurement (Actually not used, only for plotting.)
        } */
        /*else{
            /*------ UKF Prediction ------*/
            //ukf.Prediction(ukf.x_f_, ukf.p_f_);
            /*------ UKF Update ------*/ 
            /*
            odom.GPSConvertor(oxts_now, oxts_pre); // Convert GPS coordinates to mileage [meter]
            ukf.GetMeasurement(odom, oxts_now); // Get measurement for update step
            //ukf.Update(ukf.x_p_, ukf.p_p_, ukf.measurements_, odom); 
            oxts_pre = oxts_now; // Update oxts_pre (sensor data)
        }
        /*------ Visualization ------*/

        NUM ++;
        user.timerCalculator(frame_timer, "Every Frame");
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Dpoint_cmpelay for replaying
        viewer.spinOnce();
    }

    return 0;
}