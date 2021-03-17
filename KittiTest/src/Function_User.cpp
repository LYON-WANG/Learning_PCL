#include "main.h"
/* \author Leo Wang */
// Customized Supporting function for pointcloud processing 
// using PCL

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
class Oxts_Data{
    public:
        Oxts_Data(){};
        float lat;   // ** GPS ** latitude of the oxts  - unit [deg]    
        float lon;   // ** GPS ** longitude of the oxts - unit [deg]
        float alt;   // ** GPS ** altitude of the oxts  - unit [m]

        float roll;  // roll angle(rad), 0 = level, positive = left side up, range : -pi   .. + pi
        float pitch; // pitch angle(rad), 0 = level, positive = front down, range : -pi / 2 .. + pi / 2
        float yaw;   // heading(rad), 0 = east, positive = counter clockwise, range : -pi   .. + pi

        float vn; // velocity towards north [m/s]
        float ve; // velocity towards east [m/s]
        float vf; // forward velocity, i.e.parallel to earth - surface [m/s]
        float vl; // leftward velocity, i.e.parallel to earth - surface [m/s]
        float vu; // upward velocity, i.e.perpendicular to earth - surface [m/s]

        float ax; // acceleration in x, i.e.in direction of vehicle front [m/s^2]
        float ay; // acceleration in y, i.e.in direction of vehicle left [m/s^2]
        float az; // acceleration in z, i.e.in direction of vehicle top [m/s^2]
        float af; // forward acceleration [m/s^2]
        float al; // leftward acceleration [m/s^2]
        float au; // upward acceleration [m/s^2]

        float wx; // angular rate around x [rad/s]
        float wy; // angular rate around y [rad/s]
        float wz; // angular rate around z [rad/s]
        float wf; // angular rate around forward axis [rad/s]
        float wl; // angular rate around leftward axis [rad/s]
        float wu; // angular rate around upward axis [rad/s]

};

template<typename PointT>
std::tuple<std::vector<std::string>, int16_t> 
 User<PointT>::loadFile (const std::string &folderPath){
    // Count the total number of files in the path and return the path of all files.
    std::vector<std::string> filePaths; 
    DIR *path;
    struct dirent *ep;
    char path_array[(int) folderPath.length() + 1];
    strcpy(path_array, folderPath.c_str());
    path = opendir(path_array);
    int16_t count = 0;
    if(path != NULL){
        while(ep = readdir(path)){
            if(!ep -> d_name || ep -> d_name[0] == '.')
                continue;
            filePaths.push_back(folderPath + ep -> d_name);
            count ++;
        }
        (void) closedir(path);
        std::sort(filePaths.begin(), filePaths.end());
    }
    else
        perror("Couldn't open the directory...");
    std::cout << "Found " << count << " files in folder [" << folderPath << "]."<< std::endl;
    return std::make_tuple(filePaths, count);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
 User<PointT>::loadKitti (const std::vector<std::string> &filePaths, 
                          const int16_t &NUM){
    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    fstream input(filePaths[NUM].c_str(), ios::in | ios::binary);
    if(!input.good()){
		cerr << "Could not read file: " << filePaths[NUM] << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
    int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
    pcl::copyPointCloud(*points, *cloud);
    std::cout << "Load file: [" << filePaths[NUM] << "]." << std::endl;
    return cloud;
}

template<typename PointT>
void 
 User<PointT>::timerCalculator (const std::chrono::_V2::system_clock::time_point &start_time,
                                const std::string &function){
    // Should use "auto start_fast = std::chrono::system_clock::now()" to start timer.
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * 
            std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    std::cout << "[--Timer--] " << function <<" time used: " << time_passed << " [s]." << std::endl;
    //return time_passed; // [seconds]
}

template<typename PointT>
typename pcl::PCLPointCloud2::Ptr
 User<PointT>::loadPCD(const std::vector<std::string> &filePaths, 
                          const int16_t &NUM){
    // Load .pcd file
    pcl::PCDReader fileReader;
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    fileReader.read(filePaths[NUM], *cloud);
    std::cout << "Load file: [" << filePaths[NUM] << "]." << std::endl;
    return cloud;
}

template<typename PointT>
void 
 User<PointT>::initCamera (pcl::visualization::PCLVisualizer &viewer,
                           const Color &background_color, 
                           const CameraAngle &camera_angle){
    viewer.setBackgroundColor(background_color.R, background_color.G, background_color.B); // Set black background
    viewer.initCameraParameters();
    const int distance = 90;
    if(camera_angle != FPS)
        viewer.addCoordinateSystem(1.0);
    switch(camera_angle) {
        case TOP:
        viewer.setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case SIDE:
        viewer.setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS:
        viewer.setCameraPosition(-10, 0, 0, 0, 0, 1); break;
    }
}

template<typename PointT>
void 
 User<PointT>::showPointcloud (pcl::visualization::PCLVisualizer &viewer, 
                               typename pcl::PointCloud<PointT>::Ptr &cloud, 
                               const int &point_size,
                               const Color &color, 
                               const std::string &name){
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(cloud, color.R, color.G, color.B);
    viewer.addPointCloud(cloud, name);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.R, color.G, color.B, name);
}

template<typename PointT>
void
 User<PointT>::drawBoundingBox (pcl::visualization::PCLVisualizer &viewer, Box box, 
                                int box_id, Color color, 
                                float opacity){
    auto box_length = box.x_max-box.x_min;
    auto box_width = box.y_max-box.y_min;
    auto box_height = box.z_max-box.z_min;
    const float ratio = 3.5;
    if(box_height < 4 && box_width < 6 && box_length < 6 &&
       box_height/box_length < ratio && box_height/box_width < ratio && box_length/box_width < ratio && box_length/box_height < ratio &&
       box_width/box_length < ratio && box_width/box_length < ratio){
        std::string cube_ID = "box" + std::to_string(box_id);
        viewer.addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.R, color.G, color.B, cube_ID);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube_ID); 
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.R, color.G, color.B, cube_ID);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube_ID);
        std::string cube_fill = "fill" + std::to_string(box_id);
        viewer.addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.R, color.G, color.B, cube_fill);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube_fill); 
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.R, color.G, color.B, cube_fill);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.5, cube_fill);
    }
}

