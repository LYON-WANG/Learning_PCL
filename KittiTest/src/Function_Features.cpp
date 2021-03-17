#include "main.h"
/* \author Leo Wang */
// Customized Registration function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      03/2020
 */

template<typename PointT>  // Estimate normal of point cloud
 typename pcl::PointCloud<pcl::PointNormal>::Ptr 
 Features<PointT>::Normal_Estimation( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
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