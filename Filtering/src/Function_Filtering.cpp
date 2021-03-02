#include "Filtering.h"
/* \author Leo Wang */
// Customized Filtering function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::PassThroughFilter(  const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                        const std::__cxx11::string &axis, const std::array<float, 2> &limits){
    // Create filtered object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::PassThrough<PointT> passFilter;
    passFilter.setInputCloud(cloud);  // Set input point cloud 
    passFilter.setFilterFieldName(axis); // Set filter axis
    passFilter.setFilterLimits(limits[0], limits[1]); // Set the filter acceptable range
    passFilter.filter(*cloud_filtered);
    std::cout << "[PassFilter " << axis << "(" << limits[0] << " -> " << limits[1] << ") ] " << " Original points: " 
              << cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::VoxelGridDownSampling( const typename pcl::PCLPointCloud2::Ptr &cloud2, const float &filterRes){
    // DownSample the dataset using a given leaf size
    pcl::PCLPointCloud2::Ptr cloud2_filtered(new pcl::PCLPointCloud2()); // Create filtered object
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
    voxelFilter.setInputCloud(cloud2); // Set input point cloud
    voxelFilter.setLeafSize(filterRes, filterRes, filterRes); // Set voxel size
    voxelFilter.filter(*cloud2_filtered);
    pcl::fromPCLPointCloud2(*cloud2_filtered, *cloud_filtered); // PCLPointCloud2 ---> pcl::PointXYZ
    std::cout << "[VoxelGridDownSampling]  Original points: " << cloud2->width * cloud2->height <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
} // this function structure looks like a shit.... ,(O∆O),

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::StatisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                            const int &meanK, const double &StddevMulThresh){
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK); // Set the number of nearest neighbors to use for mean distance estimation.
    sor.setStddevMulThresh(StddevMulThresh); // Set threshold for determining outliers [smaller -> more stringent]
    // sor.setNegative(true);
    sor.filter(*cloud_filtered);
    std::cout << "[StatisticalOutlierRemoval] " << " Original points: " 
              << cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
}  // Can be used for reducing noise

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::RadiusOutlierRemoval( const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                        const double &Radius, const int &MinNeighborsInRadius){
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(Radius);
    ror.setMinNeighborsInRadius(MinNeighborsInRadius);
    ror.filter(*cloud_filtered);
    std::cout << "[RadiusOutlierRemoval] " << " Original points: " 
              << cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::UniformSampling(  const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                    const float &SearchRadius){
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::UniformSampling<PointT> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(SearchRadius);
    us.filter(*cloud_filtered);
    std::cout << "[UniformSampling] " << " Original points: " 
              << cloud->points.size() <<  ", Filtered points: " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
 } // Uniform sampling: Keep one point in the radius sphere (center of gravity point)

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
 Filters<PointT>::IndicesExtract(const typename pcl::PointCloud<PointT>::Ptr &cloud, boost::shared_ptr<const PointT> &indices){
    typename pcl::PointCloud<PointT>::Ptr cloud_output(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    // ei.setNegative(true);
    ei.filter(*cloud_output);
    return cloud_output;
 }