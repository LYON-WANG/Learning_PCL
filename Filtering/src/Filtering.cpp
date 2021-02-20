#include "Filtering.h"
#include "Function_Filtering.cpp"
#include "supportFunction.cpp"

int main (int argc, char** argv)
{
  Filters<pcl::PointXYZ> filters;
  // Read test dataset and create Pointcloud object
  pcl::PCLPointCloud2::Ptr cloud2_ptr(new pcl::PCLPointCloud2());
  pcl::PCDReader fileReader;
  fileReader.read("../../Test_data/data_1/0000000000.pcd", *cloud2_ptr);
  if(cloud2_ptr==NULL) { std::cout << "pcd file read err" << std::endl; return -1;}
  std::cout << "PointCLoud before filtering: " << cloud2_ptr->width * cloud2_ptr->height
       << ", data format: ( " << pcl::getFieldsList (*cloud2_ptr) << " )." << std::endl;

  // VoxelGrid DownSampling
  auto cloud_down_ptr = filters.VoxelGridDownSampling(cloud2_ptr, 0.1f);

  // Pass Filter
  auto cloud_pass_ptr = filters.PassThroughFilter(cloud_down_ptr, "z", std::array<float, 2> {0.0f, 1.0f});

  // Statistical Outlier Removal
  auto cloud_SOR_filtered = filters.StatisticalOutlierRemoval(cloud_down_ptr, 50, 1.0d);

  // Radius Outlier Removal
  auto cloud_ROR_filtered = filters.RadiusOutlierRemoval(cloud_down_ptr, 1.2, 2);

  //Uniform Sampling
  auto cloud_Uniform_filtered = filters.UniformSampling(cloud_down_ptr, 0.2);  

  // Visualization
  pcl::visualization::CloudViewer viewer("PCD Viewer");
  viewer.showCloud(cloud_Uniform_filtered);
  while (!viewer.wasStopped()){} // Do nothing but wait
  return 0;
}