#include "Segmentation.h"
/* \author Leo Wang */
// Customized Filtering function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */

template<typename PointT>
std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
 Segmentation<PointT>::SeparateClouds(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                      const pcl::PointIndices::Ptr &inliers) {
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_other(new pcl::PointCloud<PointT>());
    // Copy inliers point cloud as plane
    for (int index : inliers -> indices) {
        cloud_plane -> points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> EI;
    // Extract the inliers so that we can get obstacles point cloud
    EI.setInputCloud(cloud);
    EI.setIndices(inliers);
    EI.setNegative(true);
    EI.filter(*cloud_other);
    return std::make_tuple(cloud_plane, cloud_other);;
} // Should combine these two functions...

template<typename PointT>
std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
 Segmentation<PointT>::PlaneSegmentation(const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                         const int &maxIterations, 
                                         const float &distanceThreshold) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // defeine plane inliers
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); // Set plane model
    seg.setMethodType (pcl::SAC_RANSAC); // Method: RANSAC
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold (distanceThreshold); 
    seg.setInputCloud (cloud); // Input
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_other(new pcl::PointCloud<PointT>());
    std::tie(cloud_plane, cloud_other) = SeparateClouds(cloud, inliers);
    std::cout << "Plane points: " << cloud_plane->points.size() << ", other points: " 
              << cloud_other->points.size() << std::endl;
    return std::make_tuple(cloud_plane, cloud_other);
 }
