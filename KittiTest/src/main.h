/* \author Leo Wang */
// Real-time object-detection, plane segmentation and registration test
// based on KITTI dataset and using PCL

/**
 * Developer: Leo Wang & Varun Hegde
 * E-mail:    liangyu@student.chalmers.se
              varunh@student.chalmers.se
 * Date:      03/2020
 */
#ifndef MAIN_H_
#define MAIN_H_

#include <iostream>
//#include <fstream>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <dirent.h>
#include <memory>
#include <random>
#include <string>
#include <thread>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/LU>

// PCL Library
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/common/io.h>
#include <pcl/point_types.h>
//#include <pcl/common/point_operators.h>

// Filters
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h> // Extract pointCloud according to indices
#include <pcl/filters/passthrough.h>     // PassThrough Filter
#include <pcl/filters/radius_outlier_removal.h>      // Radius Outlier Removal
#include <pcl/filters/statistical_outlier_removal.h> // Statistical Outlier Removal
#include <pcl/filters/uniform_sampling.h>            // Uniform Sampling
#include <pcl/filters/voxel_grid.h>                  // VoxelGrid Down Sampling

// Features
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

// Segmentation
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>

// Registration
#include <pcl/registration/gicp.h>   // ICP plane-to-plane
#include <pcl/registration/icp.h>    // ICP point-to-point
#include <pcl/registration/icp_nl.h> // ICP Nolinear point-to-plane
#include <pcl/registration/ndt.h>    // NDT Registration
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transforms.h> // Transformation matrix

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

// Matplotlib cpp
#include "matplotlibcpp.h"

#define RED Color(0.6, 0, 0)
//#define RED    Color(0.863, 0.078, 0.235)
#define GREEN Color(0.235, 0.702, 0.443)
#define BLUE Color(0.4, 0.698, 1)
#define VIOLET Color(0.933, 0.510, 0.933)
#define BLACK Color(0, 0, 0)
#define WHITE Color(1, 1, 1)

struct Box {
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

bool point_cmp(pcl::PointXYZ a, pcl::PointXYZ b) { return a.z < b.z; };

enum CameraAngle { TOP, FPS, SIDE };

struct Color {
  float R, G, B;

  Color(float setR, float setG, float setB) : R(setR), G(setG), B(setB) {}
};

class Oxts_Data {
public:
  Oxts_Data(){};
  double lat; // ** GPS ** latitude of the oxts  - unit [deg]
  double lon; // ** GPS ** longitude of the oxts - unit [deg]
  double alt; // ** GPS ** altitude of the oxts  - unit [m]

  double roll;  // roll angle(rad), 0 = level, positive = left side up, range :
                // [-pi .. + pi]
  double pitch; // pitch angle(rad), 0 = level, positive = front down, range :
                // [-pi/2 .. + pi/2]
  double yaw;   // heading(rad), 0 = east, positive = counter clockwise, range :
                // [-pi .. + pi]

  double vn; // velocity towards north [m/s]
  double ve; // velocity towards east [m/s]
  double vf; // forward velocity, i.e.parallel to earth - surface [m/s]
  double vl; // leftward velocity, i.e.parallel to earth - surface [m/s]
  double vu; // upward velocity, i.e.perpendicular to earth - surface [m/s]

  double ax; // acceleration in x, i.e.in direction of vehicle front [m/s^2]
  double ay; // acceleration in y, i.e.in direction of vehicle left [m/s^2]
  double az; // acceleration in z, i.e.in direction of vehicle top [m/s^2]
  double af; // forward acceleration [m/s^2]
  double al; // leftward acceleration [m/s^2]
  double au; // upward acceleration [m/s^2]

  double wx; // angular rate around x [rad/s]
  double wy; // angular rate around y [rad/s]
  double wz; // angular rate around z [rad/s]
  double wf; // angular rate around forward axis [rad/s]
  double wl; // angular rate around leftward axis [rad/s]
  double wu; // angular rate around upward axis [rad/s]
};

/*---------------------------------------------------------------------------*/
template <typename PointT> class Filters {
public:
  // Constructor
  Filters() = default;

  // Destructor
  ~Filters() = default;
  typename pcl::PointCloud<PointT>::Ptr
  PassThroughFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                    const std::string &axis,
                    const std::array<float, 2> &limits);
  typename pcl::PointCloud<PointT>::Ptr
  VoxelGridDownSampling(const typename pcl::PCLPointCloud2::Ptr &cloud2,
                        const float &filterRes);
  typename pcl::PointCloud<PointT>::Ptr
  StatisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                            const int &meanK, const double &StddevMulThresh);
  typename pcl::PointCloud<PointT>::Ptr
  RadiusOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                       const double &Radius, const int &MinNeighborsInRadius);
  typename pcl::PointCloud<PointT>::Ptr
  UniformSampling(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                  const float &SearchRadius);
  typename pcl::PointCloud<PointT>::Ptr
  boxFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
            const Eigen::Vector4f &min_point, const Eigen::Vector4f &max_point,
            const bool &setNegative = false);
};
/*---------------------------------------------------------------------------*/
template <typename PointT> class Features {
public:
  // Constructor
  Features() = default;

  // Destructor
  ~Features() = default;

  typename pcl::PointCloud<pcl::PointNormal>::Ptr
  Normal_Estimation(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                    const int &KSearch); // Estimate normal of point cloud
};
/*---------------------------------------------------------------------------*/
template <typename PointT> class Segmentation {
public:
  // Constructor
  Segmentation() = default;

  // Destructor
  ~Segmentation() = default;

  typename pcl::PointCloud<PointT>::Ptr
  indicesExtract(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                 pcl::PointIndices::Ptr &indices,
                 const bool &set_negative = false);
  typename pcl::PointCloud<PointT>::Ptr
  RoughGroundExtraction(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                        const float &height_threshold, const int &min_number);
  std::tuple<typename pcl::PointCloud<PointT>::Ptr,
             typename pcl::PointCloud<PointT>::Ptr>
  PlaneSegmentationRANSAC(
      const typename pcl::PointCloud<PointT>::Ptr &original_cloud,
      const typename pcl::PointCloud<PointT>::Ptr &rough_ground_cloud,
      const int &maxIterations, const float &distanceThreshold);
  std::tuple<typename pcl::PointCloud<PointT>::Ptr,
             typename pcl::PointCloud<PointT>::Ptr>
  ProgressiveMorphologicalSegmentation(
      const typename pcl::PointCloud<PointT>::Ptr &cloud,
      const int &MaxWindowSize, const float &Slope,
      const float &InitialDistance, const float &MaxDistance);
  std::vector<typename pcl::PointCloud<PointT>::Ptr>
  EuclideanClustering(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                      const float &ClusterTolerance, const int &MinSize,
                      const int &MaxSize);
  Box findBoundingBox(const typename pcl::PointCloud<PointT>::Ptr &cluster);
};
/*---------------------------------------------------------------------------*/
template <typename PointT> class Registration {
public:
  // Constructor
  Registration() = default;

  // Destructor
  ~Registration() = default;

  void print4x4Matrix(const Eigen::Matrix4f &
                          matrix); // Print Rotation Matrix & Translation Vector
  std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f>
  ICP_Point2Point(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                  const typename pcl::PointCloud<PointT>::Ptr &cloud_target,
                  const Eigen::Matrix4f &init_transform,
                  const int &MaxIteration, const float &Epsilon,
                  const float &MaxCorrespondenceDistance); // ICP point-to-point
  std::tuple<typename pcl::PointCloud<pcl::PointNormal>::Ptr, Eigen::Matrix4f>
  ICP_Point2Plane(
      const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source,
      typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target,
      const Eigen::Matrix4f &init_transform, const int &MaxIteration,
      const float &Epsilon,
      const float &MaxCorrespondenceDistance); // ICP point-to-plane
  std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f>
  ICP_Plane2Plane(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                  const typename pcl::PointCloud<PointT>::Ptr &cloud_target,
                  const int &MaxIteration, const float &Epsilon,
                  const float &MaxCorrespondenceDistance); // ICP plane-to-plane
  std::tuple<typename pcl::PointCloud<PointT>::Ptr, Eigen::Matrix4f>
  NDT_Registration(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                   const typename pcl::PointCloud<PointT>::Ptr &cloud_target,
                   const Eigen::Matrix4f &init_guess,
                   const float &tTransformationEpsilon, const float &StepSize,
                   const float &Resolution, const int &MaxIteration);
  std::tuple<typename pcl::PointCloud<pcl::PointNormal>::Ptr, Eigen::Matrix4f>
  SAC_IA(const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source,
         const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target,
         const float &SearchRadius, const int &MaxIteration,
         const int &NumberOfSamples); // Sample Consensus Initial Aligment
};
/*---------------------------------------------------------------------------*/
template <typename PointT> class User {
public:
  // Constructor
  User() = default;

  // Destructor
  ~User() = default;

  std::tuple<std::vector<std::string>, int16_t>
  loadFile(const std::string &folderPath);
  typename pcl::PointCloud<PointT>::Ptr
  loadKitti(const std::vector<std::string> &filePaths, const int16_t &NUM);
  Oxts_Data loadOxts(const std::vector<std::string> &filePaths,
                     const int16_t &NUM);
  void
  timerCalculator(const std::chrono::_V2::system_clock::time_point &start_time,
                  const std::string &function);
  pcl::PCLPointCloud2::Ptr loadPCD(const std::vector<std::string> &filePaths,
                                   const int16_t &NUM);
  void showPointcloud(pcl::visualization::PCLVisualizer &viewer,
                      typename pcl::PointCloud<PointT>::Ptr &cloud,
                      const int &point_size, const Color &color,
                      const std::string &name);
  void initCamera(pcl::visualization::PCLVisualizer &viewer,
                  const Color &background_color,
                  const CameraAngle &camera_angle);
  void drawBoundingBox(pcl::visualization::PCLVisualizer &viewer, Box box,
                       int box_id, Color color, float opacity);
  std::vector<double> Transformmatrix_to_states(const Eigen::Matrix4f &R);
};
/*---------------------------------------------------------------------------*/
#endif /* MAIN_H_ */