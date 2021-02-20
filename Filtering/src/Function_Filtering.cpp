#include "Filtering.h"
//#include "supportFunction.cpp"
/* \author Leo Wang */
// Customized Filtering function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::PassThroughFilter(  const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                                                    const std::__cxx11::string &axis, const std::array<float, 2> &limits){
    // Create filtered object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::PassThrough<PointT> passFilter;
    passFilter.setInputCloud(cloud);  // Set input point cloud 
    passFilter.setFilterFieldName(axis); // Set filter axis
    passFilter.setFilterLimits(limits[0], limits[1]); // Set the filter acceptable range
    passFilter.filter(*cloud_filtered);
    std::cout << "[PassFilter " << axis << "(" << limits[0] << ", " << limits[1] << ") ] " << " Original points: " 
              << cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::VoxelGridDownSampling( const typename pcl::PCLPointCloud2::Ptr &cloud2, const float &filterRes){
    // DownSample the dataset using a given leaf size
    pcl::PCLPointCloud2::Ptr cloud2_filtered(new pcl::PCLPointCloud2()); // Create filtered object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
    voxelFilter.setInputCloud(cloud2); // Set input point cloud
    voxelFilter.setLeafSize(filterRes, filterRes, filterRes); // Set voxel size
    voxelFilter.filter(*cloud2_filtered);
    pcl::fromPCLPointCloud2(*cloud2_filtered, *cloud_filtered);
    std::cout << "[VoxelGridDownSampling]  Original points: " << cloud2->width * cloud2->height <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
} // Need to improve code structue......

