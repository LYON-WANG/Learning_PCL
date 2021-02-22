/* \author Leo Wang */
// Customized Segmentation function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>
#include <dirent.h>

// Eigen
//#include <Eigen/Dense>

// PCL Library
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
template<typename PointT>
class Segmentation{
public:
    // Constructor
    Segmentation() = default;

    // Destructor
    ~Segmentation() = default;
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(const typename pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers);
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> PlaneSegmentation(const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &maxIterations, const float &distanceThreshold);
};
#endif /* SEGMENTATION_H_ */