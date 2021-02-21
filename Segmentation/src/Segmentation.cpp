#include "Segmentation.h"

int main(int argc, char** argv){
    // Create new PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Create random pointcloud
    cloud->width  = 15;
    cloud->height = 1;//无序点云
    cloud->points.resize (cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1.0; 
    }
    // Set some outliers
    cloud->points[0].z = 2.0;
    cloud->points[3].z = -2.0;
    cloud->points[6].z = 4.0;
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // set plane model
    seg.setMethodType(pcl::SAC_RANSAC); // RANSAC method
    seg.setDistanceThreshold(0.01); // Threshold to determine whether on the plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients); // Get the plane coefficient and the index of the point on the plane

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    } 
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                    << cloud->points[inliers->indices[i]].y << " "
                                    << cloud->points[inliers->indices[i]].z << std::endl;

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (255, 255, 255); // Set white
    viewer.initCameraParameters ();

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setIndices (boost::make_shared<const pcl::PointIndices> (*inliers));
    extract_indices.setInputCloud (cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
    extract_indices.filter (*output);
    std::cerr << "output point size : " << output->points.size () << std::endl;

    // Set inliers Red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_handler(output, 255, 0, 0);
    viewer.addPointCloud(output, output_handler, "plan point");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "plan point");
    // Set outliers Green
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>);
    extract_indices.setNegative (true);
    extract_indices.filter(*cloud_other);
    std::cerr << "other point size : " << cloud_other->points.size () << std::endl;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_other_handler(cloud_other, 0, 255, 0);
    viewer.addPointCloud (cloud_other, cloud_other_handler, "other point");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "other point");

    while (!viewer.wasStopped()){
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}