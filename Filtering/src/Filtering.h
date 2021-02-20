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
#include <pcl/filters/voxel_grid.h>         // VoxelGrid DownSampling

#include <pcl/visualization/cloud_viewer.h> // Visualization

template<typename PointT>
class Filters{
public:
    // Constructor
    Filters() = default;

    // Destructor
    ~Filters() = default;
    typename pcl::PointCloud<PointT>::Ptr PassThroughFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::__cxx11::string &axis, const std::array<float, 2> &limits);
    typename pcl::PointCloud<PointT>::Ptr VoxelGridDownSampling( const typename pcl::PCLPointCloud2::Ptr &cloud, const float &filterRes);
    typename pcl::PointCloud<PointT>::Ptr StatisticalOutlierRemoval();
    
};

#endif /* FILTERING_H_ */