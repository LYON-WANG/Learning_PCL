#include "Registration.h"
// Customized
#include "Filtering.h"
#include "Segmentation.h"
#include "Function_Segmentation.cpp"
#include "Function_Registration.cpp"
#include "Function_Filtering.cpp"
#include "supportFunction.cpp"

bool point_cmp(pcl::PointXYZ a, pcl::PointXYZ b){
    return a.z<b.z;
}

int main(int argc, char** argv){
    Filters<pcl::PointXYZ> filter;
    Segmentation<pcl::PointXYZ> segmentation;
    Registration<pcl::PointXYZ> registration;
    pcl::PCLPointCloud2::Ptr cloud_source(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_target(new pcl::PCLPointCloud2());
    pcl::PCDReader fileReader;
    fileReader.read("../../Test_data/data_2/0000000000.pcd", *cloud_source);
    fileReader.read("../../Test_data/data_2/0000000001.pcd", *cloud_target);
    if(cloud_source == NULL || cloud_target == NULL){ 
        std::cout << "Source pcd file read err" << std::endl; return -1;}

    /*------ 1. Down Sampling ------*/
    auto timer_preprocess = std::chrono::system_clock::now(); // Start timer
    std::cout << "...Preprocessing..." << std::endl;
    auto cloud_source_down = filter.VoxelGridDownSampling(cloud_source, 0.4f);
    timer_calculate(timer_preprocess, "Down Sampling");

    auto cloud_target_down = filter.VoxelGridDownSampling(cloud_target, 0.4f);

    timer_calculate(timer_preprocess, "Down Sampling");

    /*------ 2. Statistical Outlier Removal ------*/
    auto timer_outlier = std::chrono::system_clock::now(); // Start timer
    cloud_source_down = filter.StatisticalOutlierRemoval(cloud_source_down, 30, 1.0d);
    timer_calculate(timer_outlier, "Outlier Removal");

    /*------ 3. Pass Filter ------*/
    auto cloud_source_pass = filter.PassThroughFilter(cloud_source_down, "z", std::array<float, 2> {-3.0f, -1.0f});
    // Resort points in Z axis
    std::sort(cloud_source_pass->points.begin(),cloud_source_pass->points.end(),point_cmp);
    timer_calculate(timer_preprocess, "Total PreProcessing");
   
    /*------ 4. Plane Segmentation ------*/
    auto timer_plane = std::chrono::system_clock::now(); // Start timer
    std::cout << "\n ...Plane Segmentation..." << std::endl;
    // Rought plane extraction
    auto RoughGroundPoints = segmentation.RoughGroundExtraction(cloud_source_pass, 1.0, 30);
    timer_calculate(timer_plane, "Rough Ground Extraction");
    // RANSAC Segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
    auto start_timer = std::chrono::system_clock::now();
    std::tie(cloud_plane, cloud_other) = segmentation.PlaneSegmentation(RoughGroundPoints, 100, 0.3);
    timer_calculate(timer_plane, "Plane Segmentation");
    timer_calculate(timer_preprocess, "Total plane Segmentation");

    // Estimate normals
    std::cout << "\n...Estimate normals..." << std::endl;
    auto timer_normal = std::chrono::system_clock::now(); // Start timer
    auto cloud_source_normal = registration.Normal_Estimation(cloud_source_down, 30);
    auto cloud_target_normal = registration.Normal_Estimation(cloud_target_down, 30);
    timer_calculate(timer_normal, "Normal Estimation");

    // ICP:
    std::cout << "\n...Registration..." << std::endl;
    auto timer_ICP = std::chrono::system_clock::now(); // Start timer
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointNormal>());
    Eigen::Matrix4f TransMatrix;
    std::tie(cloud_aligned, TransMatrix) = registration.ICP_Point2Plane(cloud_source_normal, cloud_target_normal, 100, 1e-6, 0.2);
    // std::tie(cloud_aligned, TransMatrix) = registration.ICP_Plane2Plane(cloud_source_down, cloud_target_down, 100, 1e-6, 0.1);
    // auto cloud_aligned = registration.NDT_Registration(cloud_source_down, cloud_target_down);
    //std::tie(cloud_aligned, TransMatrix) = registration.SAC_IA(cloud_source_normal, cloud_target_normal, 0.05, 100, 3);
    timer_calculate(timer_ICP, "ICP");

    // Visualization 
    pcl::visualization::PCLVisualizer viewer("ICP TEST");
    viewer.setBackgroundColor(255, 255, 255); // Set black background
    viewer.initCameraParameters();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_plane_h(cloud_plane, 0, 250, 0);
    viewer.addPointCloud(cloud_plane, cloud_plane_h, "cloud plane");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_other_h(cloud_other, 180, 20, 20);
    viewer.addPointCloud(cloud_other, cloud_other_h, "cloud other");
    //viewer.addPointCloud(cloud_ICP, cloud_icp_color_h, "cloud_icp");
    //viewer.addText("ICP result", 10, 15, 16, 0.0, 0.0, 0.0, "cloud_icp");
    while(!viewer.wasStopped()){
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}