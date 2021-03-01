/* \author Leo Wang */
// Customized Registration function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
#ifndef REGISTRATION_H_
#define REGISTRATION_H_

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

// Filters
#include <pcl/filters/passthrough.h>        // PassThrough Filter
#include <pcl/filters/voxel_grid.h>         // VoxelGrid Down Sampling
#include <pcl/filters/statistical_outlier_removal.h> // Statistical Outlier Removal
#include <pcl/filters/radius_outlier_removal.h>      // Radius Outlier Removal
#include <pcl/filters/uniform_sampling.h>            // Uniform Sampling
#include <pcl/filters/extract_indices.h>             // Extract pointCloud according to indices

// Features
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

// Segmentation

// Registration
#include <pcl/registration/icp.h>  // ICP point-to-point
#include <pcl/registration/icp_nl.h> // ICP Nolinear point-to-plane
#include <pcl/registration/gicp.h> // ICP plane-to-plane
#include <pcl/registration/ndt.h> // NDT Registration

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

template<typename PointT>
class Registration{
public:
    // Constructor
    Registration() = default;
    
    // Destructor
    ~Registration() = default;
     
    void print4x4Matrix(const Eigen::Matrix4d &matrix); // Print Rotation Matrix & Translation Vector
    typename pcl::PointCloud<pcl::PointNormal>::Ptr Normal_Estimation(const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &KSearch); // Estimate normal of point cloud
    typename pcl::PointCloud<PointT>::Ptr ICP_Point2Point(const typename pcl::PointCloud<PointT>::Ptr &cloud_source, const typename pcl::PointCloud<PointT>::Ptr &cloud_target, const int &MaxIteration); //ICP point-to-point
    typename pcl::PointCloud<pcl::PointNormal>::Ptr ICP_Point2Plane(const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source, const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target, const int &MaxIteration); //ICP point-to-plane
    typename pcl::PointCloud<PointT>::Ptr ICP_Plane2Plane(const typename pcl::PointCloud<PointT>::Ptr &cloud_source, const typename pcl::PointCloud<PointT>::Ptr &cloud_target, const int &MaxIteration); //ICP plane-to-plane
    typename pcl::PointCloud<PointT>::Ptr NDT_Registration( const typename pcl::PointCloud<PointT>::Ptr &cloud_source, const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
                                                            const float &tTransformationEpsilon,
                                                            const float &StepSize,
                                                            const float &Resolution,
                                                            const int &MaxIteration);
};
#endif /* REGISTRATION_H_ */