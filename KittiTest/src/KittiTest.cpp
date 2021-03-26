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

    /*------ Load files ------*/
    const std::string pcd_path = "../../Test_data/2011_09_26/2011_09_26_drive_0117_sync/velodyne_points/data/"; // Point cloud data path
    const std::string oxts_path = "../../Test_data/2011_09_26/2011_09_26_drive_0117_sync/oxts/data/"; // IMU&GPS data path
    int16_t fileNum;
    std::vector<std::string> pcd_paths;
    std::vector<std::string> oxts_paths;
    std::tie(pcd_paths, fileNum) = user.loadFile(pcd_path); // Load file path
    std::tie(oxts_paths, fileNum) = user.loadFile(oxts_path); // Load file path
    
    // Loop through all files
    int16_t NUM = 0;
    Oxts_Data oxts_now;
    Oxts_Data oxts_pre;
    std::vector<double> GPSX, GPSY, filteredX, filteredY; // For matplotlib
    CameraAngle camera_angle = FPS; // Set camera angle
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

        /*--------------- Pointcloud Processing(Temporary) ---------------*/
        const Eigen::Vector4f roof_min(-1.5, -1.7, -1, 1);
        const Eigen::Vector4f roof_max(2.6, 1.7, -0.4, 1);
        cloud = filter.boxFilter(cloud, roof_min, roof_max, true); // Remove roof outliers
        cloud = filter.PassThroughFilter(cloud, "z", std::array<float, 2> {-SENSOR_HEIGHT, 6.0f}); // 'Z' Pass filter

        /*---------------          UKF          ---------------*/
        // Initialize UKF
        if(NUM < 1){
            odom.Initialize();
            ukf.Initialize(odom, oxts_now);
            oxts_pre = oxts_now; // Initialize oxts_pre (sensor data)
            ukf.GetMeasurement(odom, oxts_now);
        }
        else{
            /*------ UKF Prediction ------*/
            ukf.Prediction(ukf.x_f_, ukf.p_f_);
            /*------ UKF Update ------*/ 
            odom.GPSConvertor(oxts_now, oxts_pre);
            ukf.GetMeasurement(odom, oxts_now);
            ukf.Update(ukf.x_p_, ukf.p_p_, ukf.measurements_, odom);
            oxts_pre = oxts_now; // Update oxts_pre (sensor data)
            //std::cout << ukf.x_f_ << std::endl;
        }

        /*------ Visualization ------*/
        if(DISPLAY == true){
            user.showPointcloud(viewer, cloud, 2, WHITE, "PCD");
            // matplot
            filteredX.push_back(ukf.x_f_(0,0));
            filteredY.push_back(ukf.x_f_(1,0));
            GPSX.push_back(ukf.measurements_(0,0));
            GPSY.push_back(ukf.measurements_(1,0));
            std::cout << "X: " << ukf.x_f_(0,0) << ", Y: " << ukf.x_f_(1,0) << std::endl;
            if(NUM % 1 == 0){
                matplotlibcpp::clf(); // Clear [matplotlib] previous plot
                matplotlibcpp::scatter(GPSX, GPSY, 8);
                matplotlibcpp::named_plot("Filtered", filteredX, filteredY, "r-"); // Filtered positions
                matplotlibcpp::title("UKF");
                matplotlibcpp::legend(); // Enable legend
                matplotlibcpp::grid(true); // Enable Grid
                matplotlibcpp::pause(0.001);  // Display plot continuously
                std::cout << "Plotting.................................." << std::endl;
            }
        }

        NUM ++;
        user.timerCalculator(frame_timer, "Every Frame");
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Delay for replaying
        viewer.spinOnce();
    }

    return 0;
}