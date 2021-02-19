#include "Filtering.hpp"
#include "Function_Filtering.cpp"

typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;

int main (int argc, char** argv)
{
  // Create Pointcloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>());

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

  // Create filter object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_ptr);
  pass.setFilterFieldName("z");     // Set filter axis
  pass.setFilterLimits(0.0f, 1.0f); // Set the filter range
  pass.filter(*cloud_filtered_ptr);

  // 
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