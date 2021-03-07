/* \author Leo Wang */
// Real-time object-detection, plane segmentation and registration test
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */
#ifndef REALTIMETEST_H_
#define REALTIMETEST_H_

#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>
#include <dirent.h>
#include<algorithm>

// Eigen
#include <Eigen/Dense>

// PCL Library
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

// Filters
#include <pcl/filters/passthrough.h>                 // PassThrough Filter
#include <pcl/filters/voxel_grid.h>                  // VoxelGrid Down Sampling
#include <pcl/filters/statistical_outlier_removal.h> // Statistical Outlier Removal
#include <pcl/filters/radius_outlier_removal.h>      // Radius Outlier Removal
#include <pcl/filters/uniform_sampling.h>            // Uniform Sampling
#include <pcl/filters/extract_indices.h>             // Extract pointCloud according to indices

// Features
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

// Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

// Registration
#include <pcl/registration/icp.h>    // ICP point-to-point
#include <pcl/registration/icp_nl.h> // ICP Nolinear point-to-plane
#include <pcl/registration/gicp.h>   // ICP plane-to-plane
#include <pcl/registration/ndt.h>    // NDT Registration
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transforms.h> // Transformation matrix 

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

#define RED   Color(1, 0, 0)
#define GREEN Color(0, 1, 0)
#define BLACK Color(0, 0, 0)
#define WHITE Color(1, 1, 1)

struct Box{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

bool point_cmp(pcl::PointXYZ a, pcl::PointXYZ b){
    return a.z<b.z;
};

enum CameraAngle {TOP, FPS, SIDE};

struct Color{
    float R, G, B;

	Color(float setR, float setG, float setB)
		: R(setR), G(setG), B(setB)
	{}
};
/*---------------------------------------------------------------------------*/
template<typename PointT>
class Filters{
public:
    // Constructor
    Filters() = default;

    // Destructor
    ~Filters() = default;
    typename pcl::PointCloud<PointT>::Ptr PassThroughFilter( const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::__cxx11::string &axis, const std::array<float, 2> &limits);
    typename pcl::PointCloud<PointT>::Ptr VoxelGridDownSampling( const typename pcl::PCLPointCloud2::Ptr &cloud2, const float &filterRes);
    typename pcl::PointCloud<PointT>::Ptr StatisticalOutlierRemoval( const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &meanK, const double &StddevMulThresh );
    typename pcl::PointCloud<PointT>::Ptr RadiusOutlierRemoval( const typename pcl::PointCloud<PointT>::Ptr &cloud, const double &Radius, const int &MinNeighborsInRadius );
    typename pcl::PointCloud<PointT>::Ptr UniformSampling( const typename pcl::PointCloud<PointT>::Ptr &cloud, const float &SearchRadius);
};
/*---------------------------------------------------------------------------*/
template<typename PointT>
class Features{
public:
    // Constructor
    Features() = default;

    // Destructor
    ~Features() = default;
    
    typename pcl::PointCloud<pcl::PointNormal>::Ptr Normal_Estimation(const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &KSearch); // Estimate normal of point cloud
};
/*---------------------------------------------------------------------------*/
template<typename PointT>
class Segmentation{
public:
    // Constructor
    Segmentation() = default;

    // Destructor
    ~Segmentation() = default;
    
    typename pcl::PointCloud<PointT>::Ptr IndicesExtract( const typename pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointIndices::Ptr &indices, const bool &set_negative = false);
    typename pcl::PointCloud<PointT>::Ptr RoughGroundExtraction( const typename pcl::PointCloud<PointT>::Ptr &cloud, const float & height_threshold, const int & min_number);
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> PlaneSegmentation( const typename pcl::PointCloud<PointT>::Ptr &original_cloud, const typename pcl::PointCloud<PointT>::Ptr &rough_ground_cloud, const int &maxIterations, const float &distanceThreshold);
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProgressiveMorphologicalSegmentation( const typename pcl::PointCloud<PointT>::Ptr &cloud, const int &MaxWindowSize, const float &Slope, const float &InitialDistance, const float &MaxDistance);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering( const typename pcl::PointCloud<PointT>::Ptr &cloud, const float &ClusterTolerance, const int &MinSize, const int &MaxSize);
    Box DrawBoundingBox( const typename pcl::PointCloud<PointT>::Ptr& cluster);
};
/*---------------------------------------------------------------------------*/
template<typename PointT>
class Registration{
public:
    // Constructor
    Registration() = default;
    
    // Destructor
    ~Registration() = default;
     
    void print4x4Matrix(const Eigen::Matrix4f &matrix); // Print Rotation Matrix & Translation Vector
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> ICP_Point2Point(const typename pcl::PointCloud<PointT>::Ptr &cloud_source, const typename pcl::PointCloud<PointT>::Ptr &cloud_target, const int &MaxIteration, const float &Epsilon, const float &MaxCorrespondenceDistance); //ICP point-to-point
    std::tuple<typename pcl::PointCloud<pcl::PointNormal>::Ptr, Eigen::Matrix4f> ICP_Point2Plane(const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source, const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target, const int &MaxIteration, const float &Epsilon, const float &MaxCorrespondenceDistance); //ICP point-to-plane
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> ICP_Plane2Plane(const typename pcl::PointCloud<PointT>::Ptr &cloud_source, const typename pcl::PointCloud<PointT>::Ptr &cloud_target, const int &MaxIteration, const float &Epsilon, const float &MaxCorrespondenceDistance); //ICP plane-to-plane
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f> NDT_Registration( const typename pcl::PointCloud<PointT>::Ptr &cloud_source, const typename pcl::PointCloud<PointT>::Ptr &cloud_target, const float &tTransformationEpsilon, const float &StepSize, const float &Resolution, const int &MaxIteration);
    std::tuple<typename pcl::PointCloud<pcl::PointNormal>::Ptr, Eigen::Matrix4f> SAC_IA(const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source, const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target, const float &SearchRadius, const int &MaxIteration, const int &NumberOfSamples); // Sample Consensus Initial Aligment
};
/*---------------------------------------------------------------------------*/
template<typename PointT>
class User{
public:
    // Constructor
    User() = default;

    // Destructor
    ~User() = default;
    
    std::tuple<std::vector<std::string>, int16_t> loadFile(const std::string &folderPath);
    void timerCalculator(const std::chrono::_V2::system_clock::time_point &start_time, const std::string &function);
    pcl::PCLPointCloud2::Ptr loadPCD(const std::vector<std::string> &filePaths, const int16_t &NUM);
    void showPointcloud(pcl::visualization::PCLVisualizer &viewer, typename pcl::PointCloud<PointT>::Ptr &cloud, const int &point_size, const Color &color, const std::string &name);
    void initCamera(pcl::visualization::PCLVisualizer &viewer, const Color &background_color, const CameraAngle &camera_angle);
};

#endif /* REALTIMETEST_H_ */