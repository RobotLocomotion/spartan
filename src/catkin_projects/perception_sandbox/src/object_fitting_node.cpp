#include <ros/ros.h>
#include <unistd.h>
#include <mutex>
#include <string>

#include "Eigen/Dense"

// For argument parsing
#include <gflags/gflags.h>

// PCL specific includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

// OpenCV utilities for viz
#include <opencv2/core/eigen.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "common_utils/cv_utils.h"
#include "common_utils/pcl_utils.h"
#include "common_utils/system_utils.h"

#include "RemoteTreeViewerWrapper.hpp"

/**
- Load in PCD-format XYZRGBNormal point cloud
- Clean up with voxel grid filter
- Build freespace occupancy grid? (spiral 2)
- (publish that cloud for viz in remote tree viewer)

- Also load a list of object meshes to try to fit
- Marginal point cloud = full point cloud
- Until converged:
   - Pick a random object and seed it at a random pose
   - Maybe pick a random segment of the point cloud? (spiral 2)
   - Run ICP on that object and the current marginal point cloud
   - If fit good enough, remove matched points from marginal point cloud
   - Stop if marginal point cloud contains too few points

**/

DEFINE_string(input_pointcloud_file, "REQUIRED",
              "Point cloud file to fit objects to.");
DEFINE_string(
    input_object_list, "REQUIRED",
    "YAML file listing (unarticulated, for now) object URDFs to fit.");
DEFINE_double(leaf_size, 0.01,
              "Leaf size for voxel grid filter cleanup of input point cloud.");

using namespace std;

static const string OPENCV_WINDOW_NAME = "Point Cloud Viz Window";

void loadAndCleanTargetCloud(
    const std::string pointcloud_file, double leaf_size,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_cloud) {
  // Load input point cloud
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(FLAGS_input_pointcloud_file,
                                                   *input_cloud) ==
      -1)  //* load the file
  {
    printf("Couldn't read file %s\n", FLAGS_input_pointcloud_file.c_str());
    exit(-1);
  }

  // Voxel grid filter to clean up redundant points and de-noise
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*target_cloud);
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "object_fitting_node");

  RemoteTreeViewerWrapper rm;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  loadAndCleanTargetCloud(FLAGS_input_pointcloud_file, FLAGS_leaf_size,
                          target_cloud);

  rm.publishPointCloud(convertPclPointXyzEtcToMatrix3Xd<pcl::PointXYZRGBNormal>(target_cloud),
                       {"object_fitting_node", "pc"},
                       convertPclPointRgbEtcToVectorOfVectors<pcl::PointXYZRGBNormal>(target_cloud));
  cv::destroyAllWindows();
  return 0;
}