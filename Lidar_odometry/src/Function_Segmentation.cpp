#include "main.h"
/* \author Leo Wang */
// Customized Segmentation function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Segmentation<PointT>::indicesExtract(const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                 pcl::PointIndices::Ptr &indices,
                                 const bool &set_negative){
    typename pcl::PointCloud<PointT>::Ptr cloud_output(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.setNegative(set_negative);
    ei.filter(*cloud_output);
    return cloud_output;
 }

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Segmentation<PointT>::RoughGroundExtraction (const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                              const float & height_threshold, 
                                              const int & min_number){
    typename pcl::PointCloud<PointT>::Ptr cloud_output(new pcl::PointCloud<PointT>());
    float sum = 0.0f; 
    int num = 0;         
    for(int i = 0; (i < cloud->points.size()) && (num < min_number); i++){
        sum += cloud->points[i].z; 
        num ++;
    } 
    // Calulate average point height
    float average_height = num != 0 ? sum/num : 0;
    for(int i = 0; i < cloud->points.size(); i++){
        if(cloud->points[i].z < average_height + height_threshold){
            cloud_output->points.push_back(cloud->points[i]);
        }
    }
    // std::cout << "Original points: " << cloud->points.size() << ", Output points: " 
    //           << cloud_output->points.size() << std::endl;
    return cloud_output;
}

template<typename PointT>
std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
 Segmentation<PointT>::PlaneSegmentationRANSAC (const typename pcl::PointCloud<PointT>::Ptr &original_cloud, 
                                          const typename pcl::PointCloud<PointT>::Ptr &rough_ground_cloud, 
                                          const int &maxIterations, 
                                          const float &distanceThreshold) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // defeine plane inliers
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); // Set plane model
    seg.setMethodType (pcl::SAC_RANSAC);    // Method: RANSAC
    seg.setMaxIterations(maxIterations);    // Set maximum iteration
    seg.setDistanceThreshold (distanceThreshold); // unit [meter]
    seg.setInputCloud (rough_ground_cloud); // Segmentation input cloud
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_other(new pcl::PointCloud<PointT>());
    // Extract plane and non-plane cloud
    cloud_plane = indicesExtract(original_cloud, inliers);
    cloud_other = indicesExtract(original_cloud, inliers, true);
    std::cout << "[RANSAC Plane Segmentation] Plane points: " << cloud_plane->points.size() << ", other points: " 
              << cloud_other->points.size() << std::endl;
    return std::make_tuple(cloud_plane, cloud_other);
}

template<typename PointT>
std::tuple<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
 Segmentation<PointT>::ProgressiveMorphologicalSegmentation (const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                                             const int &MaxWindowSize, 
                                                             const float &Slope, 
                                                             const float &InitialDistance, 
                                                             const float &MaxDistance){
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_other(new pcl::PointCloud<PointT>());
    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    typename pcl::ProgressiveMorphologicalFilter<PointT> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(MaxWindowSize);
    pmf.setSlope(Slope);
    pmf.setInitialDistance(InitialDistance);
    pmf.setMaxDistance(MaxDistance);
    pmf.extract(ground->indices);
    // Extract pointcloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground);
    extract.filter(*cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_other);
    std::cout << "Plane points: " << cloud_plane->points.size() << ", other points: " 
              << cloud_other->points.size() << std::endl;
    return std::make_tuple(cloud_plane, cloud_other);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
 Segmentation<PointT>::EuclideanClustering (const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                            const float &ClusterTolerance,
                                            const int &MinSize, 
                                            const int &MaxSize){
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> cluster_indices;
    // Creating KdTree object for the search method of extraction.
    typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<PointT> EC;
    EC.setClusterTolerance(ClusterTolerance);
    EC.setMinClusterSize(MinSize);
    EC.setMaxClusterSize(MaxSize);
    EC.setSearchMethod(kdtree);
    EC.setInputCloud(cloud); // Input PointCloud
    EC.extract(cluster_indices);
    // Iteratively visit Pointcloud indexs 'cluster_indices', until all the clusters are splited
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++ it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit=it->indices.begin(); pit!=it->indices.end(); ++ pit)
            cloud_cluster -> points.push_back(cloud->points[*pit]);
        cloud_cluster -> width = cloud_cluster -> points.size ();
        cloud_cluster -> height = 1;
        cloud_cluster -> is_dense = true;
        if (cloud_cluster->width >= MinSize && cloud_cluster->width <= MaxSize)
            clusters.push_back(cloud_cluster);
    }
    return clusters;
}   

template<typename PointT>
Box 
 Segmentation<PointT>::findBoundingBox (const typename pcl::PointCloud<PointT>::Ptr& cluster) {
    // Find bounding box for one of the clusters
    PointT minPoint; 
    PointT maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    Box box{};
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
    return box;
}