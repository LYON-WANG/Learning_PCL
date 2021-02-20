#include "Filtering.h"
#include "Function_Filtering.cpp"
#include "supportFunction.cpp"

typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;

int main (int argc, char** argv)
{
  // Create Pointcloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // Create point cloud　
  cloud_ptr -> width  = 5;
  cloud_ptr -> height = 1;
  cloud_ptr -> points.resize (cloud_ptr->width * cloud_ptr->height);
  for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
  {
    cloud_ptr->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_ptr->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_ptr->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cerr << "Cloud before filtering:" << std::endl;
  for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
    std::cerr << "    " << cloud_ptr->points[i].x << " " 
                        << cloud_ptr->points[i].y << " " 
                        << cloud_ptr->points[i].z << std::endl;

  // Pass Filter
  Filters<pcl::PointXYZ> filters;
  std::array<float,2> range = {0.0, 1.0};
  auto cloud_filtered_ptr = filters.PassFilter(cloud_ptr, "z", range);

  std::cerr << "Cloud after filtering:" << std::endl;
  for (size_t i = 0; i < cloud_filtered_ptr->points.size (); ++i)
    std::cerr << "    " << cloud_filtered_ptr->points[i].x << " " 
                        << cloud_filtered_ptr->points[i].y << " " 
                        << cloud_filtered_ptr->points[i].z << std::endl;
  // Visualization
  pcl::visualization::CloudViewer viewer("PCD Viewer");
  viewer.showCloud(cloud_filtered_ptr);
  while (!viewer.wasStopped()){}
  return 0;
}