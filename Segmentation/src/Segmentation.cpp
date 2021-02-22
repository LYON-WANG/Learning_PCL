#include "Segmentation.h"
#include "Function_Segmentation.cpp"

int main(int argc, char** argv){
    Segmentation<pcl::PointXYZ> segmentation;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader fileReader;
    fileReader.read("../../Test_data/data_1/0000000000.pcd", *cloud);
    if(cloud==NULL) { std::cout << "pcd file read err" << std::endl; return -1;}
    std::cout << "PointCLoud before filtering: " << cloud->points.size()
        << ", data format: ( " << pcl::getFieldsList (*cloud) << " )." << std::endl;

    // Plane Segmentation
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>());
    std::tie(cloud_plane, cloud_other) = segmentation.PlaneSegmentation(cloud, 100, 0.15);

    
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (255, 255, 255); // Set white
    viewer.initCameraParameters ();
    // // Set inliers Red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_handler(cloud_plane, 255, 0, 0);
    viewer.addPointCloud(cloud_plane, plane_handler, "plane points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plane points");
    // // Set outliers Green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_other_handler(cloud_other, 0, 255, 0);
    viewer.addPointCloud (cloud_other, cloud_other_handler, "other points");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "other points");
    while (!viewer.wasStopped()){
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}