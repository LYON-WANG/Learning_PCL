#include "Filtering.h"
//#include "supportFunction.cpp"
/* \author Leo Wang */
// Customized Filtering function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr Filters<PointT>::PassFilter(  const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                                                    const std::__cxx11::string &axis, const std::array<float, 2> &range){
    // Create filtered Pointcloud object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    // Create filter object
    pcl::PassThrough<PointT> passFilter;
    passFilter.setInputCloud(cloud);
    passFilter.setFilterFieldName("z");     // Set filter axis
    passFilter.setFilterLimits(range[0], range[1]); // Set the filter range
    passFilter.filter(*cloud_filtered);
    std::cout << "[PassFilter:] Original: " <<  "Filtered: " << std::endl;
    return cloud_filtered;
}
