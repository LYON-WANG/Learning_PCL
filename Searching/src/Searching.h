/* \author Leo Wang */
// Customized Filtering function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
#ifndef SEARCHING_H_
#define SEARCHING_H_

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

#include <pcl/octree/octree.h>

template<typename PointT>
class Searching{
public:
    // Constructor
    Searching() = default;

    // Destructor
    ~Searching() = default;
    
};
#endif /* SEARCHING_H_ */