# Filters
Customized filter functions, which can make code much more brief and easy to understand.

**TO RUN:**
```
  mkdir build
  cd build
  cmake ..
  make
  cd ../bin
  ./Filtering
```

## Function Summary:
### PassThroughFilter:
```
auto outputCloud PassThroughFilter(inputCloud, axis, limits);
```
If the point cloud is collected by linear structured light scanning, the objects must be distributed widely along the z-direction, but the x- and y-direction distributions are within a limited range.
Using a straight-pass filter to determine the range of the point cloud in the z-axis direction can quickly cut out the outliers and realize the first rough processing.

### VoxelGridDownSampling:
```
auto outputCloud VoxelGridDownSampling(inputCloud, leafSize);
```
Reduce the number of points in the grid to ensure that the position of the center of gravity remains unchanged. At the same time, it saves the shape characteristics of the point cloud, which is very useful in improving the speed of registration, surface reconstruction, shape recognition and other algorithms.