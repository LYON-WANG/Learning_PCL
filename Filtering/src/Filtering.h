/* \author Leo Wang */
// Customized Filtering function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
#ifndef FILTERING_H_
#define FILTERING_H_

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

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>        // PassThrough Filter
#include <pcl/filters/voxel_grid.h>         // VoxelGrid Down Sampling
#include <pcl/filters/statistical_outlier_removal.h> // Statistical Outlier Removal
#include <pcl/filters/radius_outlier_removal.h>      // Radius Outlier Removal
#include <pcl/filters/uniform_sampling.h>            // Uniform Sampling

#include <pcl/visualization/cloud_viewer.h> // Visualization

template<typename PointT>
class Filters{
public:
    // Constructor
    Filters() = default;

    // Destructor
    ~Filters() = default;
    typename pcl::PointCloud<PointT>::Ptr PassThroughFilter( const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::__cxx11::string &axis, const std::array<float, 2> &limits);
    typename pcl::PointCloud<PointT>::Ptr VoxelGridDownSampling( const typename pcl::PCLPointCloud2::Ptr &cloud2, const float &filterRes);
    typename pcl::PointCloud<PointT>::Ptr StatisticalOutlierRemoval( const typename pcl::PointCloud<PointT>::Ptr &cloud,  const int &meanK, const double &StddevMulThresh );
    typename pcl::PointCloud<PointT>::Ptr RadiusOutlierRemoval( const typename pcl::PointCloud<PointT>::Ptr &cloud,  const double &Radius, const int &MinNeighborsInRadius );
    typename pcl::PointCloud<PointT>::Ptr UniformSampling( const typename pcl::PointCloud<PointT>::Ptr &cloud, const float &SearchRadius);
};

#endif /* FILTERING_H_ */