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
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

struct Box{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

template<typename PointT>
class Segmentation{
public:
    // Constructor
    Segmentation() = default;

    // Destructor
    ~Segmentation() = default;
    
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> PlaneSegmentation(const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &maxIterations, const float &distanceThreshold);
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProgressiveMorphologicalSegmentation(const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &MaxWindowSize, const float &Slope, const float &InitialDistance, const float &MaxDistance);
    typename pcl::PointCloud<PointT>::Ptr RoughGroundExtraction(const typename pcl::PointCloud<PointT>::Ptr &cloud, const float & height_threshold, const int & min_number);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering( const typename pcl::PointCloud<PointT>::Ptr &cloud, const float &ClusterTolerance, const int &MinSize, const int &MaxSize);
    Box DrawBoundingBox(const typename pcl::PointCloud<PointT>::Ptr& cluster);
};
#endif /* SEGMENTATION_H_ */