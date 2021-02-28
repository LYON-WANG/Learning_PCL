#include "Registration.h"
/* \author Leo Wang */
// Customized Registration function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
template<typename PointT> // Print Rotation Matrix & Translation Vector
 void Registration<PointT>::print4x4Matrix(const Eigen::Matrix4d &matrix){
    std::cout << "-----------------" << std::endl;
    std::cout << "Rotation matrix: " << std::endl;
    std::cout << "R = " << matrix (0, 0) << " " << matrix (0, 1) << " " << matrix (0, 2) << std::endl;
    std::cout << "    " << matrix (1, 0) << " " << matrix (1, 1) << " " << matrix (1, 2) << std::endl;
    std::cout << "    " <<matrix (2, 0) << " " << matrix (2, 1) << " " << matrix (2, 2) << std::endl;
    std::cout << "Translation Vector: " << std::endl;
    std::cout << "T = " << matrix (0, 3) << " " << matrix (1, 3) << " " << matrix (2, 3) << std::endl;
    std::cout << "-----------------" << std::endl;
}
template<typename PointT>  // Estimate normal of point cloud
 typename pcl::PointCloud<pcl::PointNormal>::Ptr Registration<PointT>::Normal_Estimation( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                                                                const int &KSearch){
    pcl::PointCloud<pcl::PointNormal>::Ptr output_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimationOMP<PointT, pcl::PointNormal> nest;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	nest.setSearchMethod(tree);
    nest.setKSearch(KSearch);
    //nest.setRadiusSearch(SearchRadius);
    nest.setInputCloud(cloud);
    nest.compute(*output_with_normals);
    return output_with_normals;
}

template<typename PointT>  //ICP point-to-point
 typename pcl::PointCloud<PointT>::Ptr Registration<PointT>::ICP_Point2Point(const typename pcl::PointCloud<PointT>::Ptr &cloud_source,
                                                                     const typename pcl::PointCloud<PointT>::Ptr &cloud_target, 
                                                                     const int &MaxIteration){
    typename pcl::PointCloud<PointT>::Ptr cloud_ICP(new pcl::PointCloud<PointT>);
    typename pcl::IterativeClosestPoint<PointT, PointT> icp;
    //icp.setMaximumIterations(100);
    //icp.setEuclideanFitnessEpsilon(1e-5);  // Convergence condition: The smaller the accuracy, the slower the convergence
	//icp.setMaxCorrespondenceDistance(0.10);
    icp.setInputSource(cloud_source);      
    icp.setInputTarget(cloud_target); 
    icp.align(*cloud_ICP);  
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    return cloud_ICP;
}

template<typename PointT>  //ICP point-to-plane
 typename pcl::PointCloud<pcl::PointNormal>::Ptr Registration<PointT>::ICP_Point2Plane(const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_source,
                                                                     const typename pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_target, 
                                                                     const int &MaxIteration){
    typename pcl::PointCloud<PointT>::Ptr cloud_ICP(new pcl::PointCloud<PointT>);
    typename pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setMaximumIterations(MaxIteration);
    icp.setEuclideanFitnessEpsilon(1e-6);  // Convergence condition: The smaller the accuracy, the slower the convergence
	icp.setMaxCorrespondenceDistance(0.10);
    icp.setInputSource(cloud_source);      
    icp.setInputTarget(cloud_target); 
    icp.align(*cloud_source);  
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    return cloud_source;
}