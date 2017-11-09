#include <ros/ros.h>
#include <unistd.h>
#include <mutex>
#include <string>

#include "Eigen/Dense"

// PCL specific includes
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

/**
   This node subscribes to a published PointCloud2 channel
   on which a structured point cloud is published.

   It displays the (reconstructed from the structured point cloud)
   RGB and depth images, and allows key input for changing the
   depth near and far planes (to reject really far returns) and
   allow saving as a PCD with XYZ RGB.
**/

using namespace std;

static const string OPENCV_WINDOW_NAME = "Point Cloud Viz Window";

class Grabber {
  ros::NodeHandle nh_;
  ros::Subscriber pc2_sub_;

  mutex data_update_lock_;
  bool cloud_valid_;
  pcl::PointCloud<pcl::PointXYZRGB> latest_cloud_;
  cv::Mat latest_rgb_image_;
  cv::Mat latest_depth_image_;
  double z_cutoff_plane_;

 public:
  Grabber() : cloud_valid_(false), z_cutoff_plane_(3.0) {
    // Subscrive to input video feed and publish output video feed
    string sub_channel = "/camera_1112170110/depth_registered/points";
    printf("Subbing to %s\n", sub_channel.c_str());
    pc2_sub_ = nh_.subscribe(sub_channel, 1, &Grabber::Pc2Cb, this);

    cv::namedWindow(OPENCV_WINDOW_NAME);
  }

  ~Grabber() { cv::destroyWindow(OPENCV_WINDOW_NAME); }

  void PrintCutoff() { printf("Cutoff: %f\n", z_cutoff_plane_); }

  void SaveLatestData() {
    string filename_prefix = getTimestampString() + "_";
    string rgb_filename = filename_prefix + "rgb.png";
    string depth_filename = filename_prefix + "depth.png";
    string pcd_filename = filename_prefix + "pc.pcd";

    data_update_lock_.lock();

    cv::imwrite(rgb_filename, latest_rgb_image_);
    cv::imwrite(depth_filename, latest_depth_image_);
    pcl::io::savePCDFileBinary(pcd_filename, latest_cloud_);

    data_update_lock_.unlock();

    printf("Saved data to file %s\n", filename_prefix.c_str());
  }

  void Update(char key_input) {
    if (cloud_valid_) {
      // Do visualization
      data_update_lock_.lock();
      auto image_grid = makeGridOfImages(
          {latest_rgb_image_,
           convertToColorMap(latest_depth_image_, z_cutoff_plane_, 0.0)},
          2, 10);
      data_update_lock_.unlock();
      cv::imshow(OPENCV_WINDOW_NAME, image_grid);

      // Handle keystrokes
      switch (key_input) {
        case 's':
          // save things
          SaveLatestData();
          break;
        case '[':
          // Bring cutoff plane closer
          z_cutoff_plane_ -= 0.1;
          PrintCutoff();
          break;
        case '{':
          z_cutoff_plane_ -= 1.0;
          PrintCutoff();
          break;
        case ']':
          // Bring cutoff plane farther
          z_cutoff_plane_ += 0.1;
          PrintCutoff();
          break;
        case '}':
          z_cutoff_plane_ += 1.0;
          PrintCutoff();
          break;
      }
    }
  }

  void Pc2Cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert to PCL data type
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);

    // Ensure we're ready to extract the rgb and depth images
    // before grabbing the lock.
    int width = pcl_pc2.width;
    int height = pcl_pc2.height;
    if (latest_rgb_image_.rows != height || latest_rgb_image_.cols != width) {
      latest_rgb_image_ = cv::Mat::zeros(height, width, CV_8UC3);
    }
    if (latest_depth_image_.rows != height ||
        latest_depth_image_.cols != width) {
      latest_depth_image_ = cv::Mat::zeros(height, width, CV_32FC1);
    }

    data_update_lock_.lock();
    pcl::fromPCLPointCloud2(pcl_pc2, latest_cloud_);

    for (int u = 0; u < height; u++) {
      for (int v = 0; v < width; v++) {
        // OpenCV defaults to bgr
        latest_rgb_image_.at<cv::Vec3b>(u, v)[2] = latest_cloud_(v, u).r;
        latest_rgb_image_.at<cv::Vec3b>(u, v)[1] = latest_cloud_(v, u).g;
        latest_rgb_image_.at<cv::Vec3b>(u, v)[0] = latest_cloud_(v, u).b;
        latest_depth_image_.at<float>(u, v) =
            Vector3dFromPclPoint(latest_cloud_(v, u)).norm();
      }
    }
    cloud_valid_ = true;
    data_update_lock_.unlock();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud2_capture_node");
  Grabber gr;
  while (1) {
    // Wait 33 ms (~30hz update rate) and watch for keystrokes
    char k = cv::waitKey(33);
    gr.Update(k);
    ros::spinOnce();
  }
  cv::destroyAllWindows();
  return 0;
}