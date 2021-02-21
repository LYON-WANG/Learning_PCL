#include "Searching.h"
#include "Function_Searching.cpp"
#include "Function_Filtering.h"
#include "Function_Filtering.cpp"

int main (int argc, char** argv)
{
    Filters<pcl::PointXYZ> filters;
    // Read test dataset and create Pointcloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>& cloud = *cloud_ptr; // For Quote
    pcl::PCDReader fileReader;
    fileReader.read("../../Test_data/data_1/0000000000.pcd", *cloud_ptr);
    if(cloud_ptr==NULL) { std::cout << "pcd file read err" << std::endl; return -1;}
    std::cout << "PointCLoud before filtering: " << cloud_ptr->size()
        << ", data format: ( " << pcl::getFieldsList (*cloud_ptr) << " )." << std::endl;

    // 1. Neighbors within voxel search:
    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (cloud_ptr);
    octree.addPointsFromInputCloud (); // PointCloud -> Octree

    pcl::PointXYZ searchPoint; // Define Searching point
    searchPoint.x = 10.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = 10.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = 10.0f * rand () / (RAND_MAX + 1.0f);

    std::vector<int> pointIdxVec;
    if(octree.voxelSearch(searchPoint,pointIdxVec))    // Return searching result to 'pointIdxVe'
	{
	    // Searching point info
	    std::cout << "Neighbors within voxel search at searchPoint(" 
	     << searchPoint.x << " " 
	     << searchPoint.y << " " 
	     << searchPoint.z << ")" 
	     << std::endl;
	    // Neighbors of voxels found
	   for (size_t i = 0; i < pointIdxVec.size (); ++i)
	   std::cout << " " << cloud.points[pointIdxVec[i]].x 
	       	     << " " << cloud.points[pointIdxVec[i]].y 
	       	     << " " << cloud.points[pointIdxVec[i]].z << std::endl;
	}

    // 2. K nearest neighbor search 
    int K = 10;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at ( "
            << searchPoint.x << " "
            << searchPoint.y << " " 
	        << searchPoint.z << ")" 
	        << std::endl;
    if(octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0){
        for(size_t i=0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << " " << cloud.points[ pointIdxNKNSearch[i] ].x 
		        << " " << cloud.points[ pointIdxNKNSearch[i] ].y 
		        << " " << cloud.points[ pointIdxNKNSearch[i] ].z 
		        << " (squared distance: "
		        << pointNKNSquaredDistance[i] << ")" << std::endl;
    }
    
    // 3. Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 25.0f * rand () / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" 
		    << searchPoint.x << " " 
		    << searchPoint.y << " " 
		    << searchPoint.z << ") with radius=" 
		    << radius << std::endl;

    if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	  {
	    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
	      std::cout << " " << cloud.points[ pointIdxRadiusSearch[i] ].x 
		        << " " << cloud.points[ pointIdxRadiusSearch[i] ].y 
		        << " " << cloud.points[ pointIdxRadiusSearch[i] ].z 
		        << " (squared distance: " 
			<< pointRadiusSquaredDistance[i] << ")" << std::endl;
	  }


    // Visualization 
    //pcl::visualization::CloudViewer viewer("PCD Viewer");
    //viewer.showCloud(cloud_Uniform_filtered);
    //while (!viewer.wasStopped()){} // Do nothing but wait
    return 0;
}