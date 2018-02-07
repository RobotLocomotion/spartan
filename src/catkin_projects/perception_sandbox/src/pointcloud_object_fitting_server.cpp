#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <mutex>
#include <string>

#include "Eigen/Dense"

// For argument parsing
#include <gflags/gflags.h>

// PCL specific includes
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// OpenCV utilities for viz
#include <opencv2/core/eigen.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <sensor_msgs/PointCloud2.h>

#include "common_utils/cv_utils.h"
#include "common_utils/math_utils.h"
#include "common_utils/pcl_utils.h"
#include "common_utils/system_utils.h"

#include "eigen_conversions/eigen_msg.h"
#include "perception_sandbox/AddPointCloudAtPose.h"
#include "perception_sandbox/FitObjectsByIcp.h"
#include "std_srvs/Empty.h"

/**
   This node provides object-in-point-cloud pose estimation
   service.

   Service calls:
    AddPointCloudAtPose
    ResetPointCloudCollection
    FitObjectsByIcp --> returns best fit candidate for set of objects
**/

using namespace std;

class ObjectFittingServer {
  ros::NodeHandle nh_;

  ros::Publisher pc2_pub_;
  double lastPublishTime_;
  ros::Publisher debug_cloud_pub_;

  ros::ServiceServer service_add_;
  ros::ServiceServer service_reset_;
  ros::ServiceServer service_fit_;

  mutex data_update_lock_;
  pcl::PointCloud<pcl::PointXYZRGBNormal> complete_cloud_;

 public:
  ObjectFittingServer(ros::NodeHandle nh)
      : nh_(nh),
        pc2_pub_(nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>>(
            "/object_fitting/complete_cloud", 1)),
        debug_cloud_pub_(nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>>(
            "/object_fitting/debug_cloud", 1)),
        lastPublishTime_(getUnixTime()),
        service_add_(nh_.advertiseService(
            "/object_fitting/AddPointCloudAtPose",
            &ObjectFittingServer::AddPointCloudAtPose, this)),
        service_reset_(
            nh_.advertiseService("/object_fitting/ResetPointClouds",
                                 &ObjectFittingServer::ResetPointClouds, this)),
        service_fit_(nh_.advertiseService("/object_fitting/FitObjectsByIcp",
                                          &ObjectFittingServer::FitObjectsByIcp,
                                          this)) {
    complete_cloud_.header.frame_id = "base";
  }
  ~ObjectFittingServer() {}

  bool AddPointCloudAtPose(
      perception_sandbox::AddPointCloudAtPose::Request& req,
      perception_sandbox::AddPointCloudAtPose::Response& res) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(req.point_cloud_with_transform.point_cloud, pcl_pc2);

    // Copy over the frame name -- assume it's consistent between
    // point clouds.

    // Convert from PointCloud2 to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_normals(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_without_normals);

    // Do integral-image normal estimation, which should be pretty quick
    // as the input point cloud is structured in a grid.
    pcl::PointCloud<pcl::Normal> normals;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud_without_normals);
    ne.compute(normals);
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_normals;
    pcl::concatenateFields(*cloud_without_normals, normals, cloud_with_normals);

    // Extract TF
    Eigen::Affine3d point_cloud_to_base_transform;
    tf::transformMsgToEigen(
        req.point_cloud_with_transform.point_cloud_to_base_transform.transform,
        point_cloud_to_base_transform);
    Eigen::Vector3d camera_origin;
    tf::pointMsgToEigen(req.point_cloud_with_transform.camera_origin,
                        camera_origin);

    if (camera_origin.norm() > 0.1) {
      printf("WARNING: CAMERA ORIGIN NOT USED YET.\n");
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> transformed_cloud;
    pcl::transformPointCloud(cloud_with_normals, transformed_cloud,
                             point_cloud_to_base_transform);

    // Box-crop it to specified region
    pcl::CropBox<pcl::PointXYZRGBNormal> cropBoxFilter(true);
    cropBoxFilter.setInputCloud(transformed_cloud.makeShared());
    cropBoxFilter.setMin(
        Eigen::Vector4f(req.minPt.x, req.minPt.y, req.minPt.z, 1.));
    cropBoxFilter.setMax(
        Eigen::Vector4f(req.maxPt.x, req.maxPt.y, req.maxPt.z, 1.));
    cropBoxFilter.filter(transformed_cloud);

    data_update_lock_.lock();
    complete_cloud_ += transformed_cloud;
    // Copy in frame info
    complete_cloud_.header.frame_id =
        req.point_cloud_with_transform.header.frame_id;

    // Apply a voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud(complete_cloud_.makeShared());
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(complete_cloud_);

    data_update_lock_.unlock();
    return true;
  }

  bool ResetPointClouds(std_srvs::Empty::Request& req,
                        std_srvs::Empty::Response& res) {
    data_update_lock_.lock();
    complete_cloud_.clear();
    complete_cloud_.header.frame_id = "base";
    data_update_lock_.unlock();
    return true;
  }

  bool FitObjectsByIcp(perception_sandbox::FitObjectsByIcp::Request& req,
                       perception_sandbox::FitObjectsByIcp::Response& res) {
    data_update_lock_.lock();

    // For each mesh...
    int n_objects = req.mesh_paths.size();
    for (int i = 0; i < n_objects; i++) {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr object_cloud(
          new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      /*
      TODO: need to sample point cloud from the mesh
      which requires recruiting my old drake / vtk tools...
      vtk probably easiest route, need to probe functionality
      pcl::io::loadOBJFile(req.mesh_paths[i].data, mesh);
      */

      // Scale as requested
      if (req.scales[i] != 1.0) {
        Eigen::Affine3d tf = Eigen::Affine3d::Identity();
        tf.matrix().block<3, 3>(0, 0) *= req.scales[i];

        pcl::transformPointCloud(*object_cloud, *object_cloud, tf);
      }

      // Randomly seed transform at random reasonable locations,
      // and then refine with ICP. Take the best one in terms of
      // ICP residual error.
      double best_cost = std::numeric_limits<double>::infinity();
      Eigen::Affine3d best_tf = Eigen::Affine3d::Identity();

      pcl::PointXYZRGBNormal minPtPcl, maxPtPcl;
      pcl::getMinMax3D(complete_cloud_, minPtPcl, maxPtPcl);
      Eigen::Vector3d minPt = Vector3dFromPclPoint(minPtPcl);
      Eigen::Vector3d maxPt = Vector3dFromPclPoint(maxPtPcl);

      pcl::PointXYZRGBNormal modelMinPtPcl, modelMaxPtPcl;
      pcl::getMinMax3D(*object_cloud, modelMinPtPcl, modelMaxPtPcl);
      Eigen::Vector3d modelCenter = (Vector3dFromPclPoint(modelMinPtPcl) +
                                     Vector3dFromPclPoint(modelMaxPtPcl)) /
                                    2.;

      for (int attempt = 0; attempt < req.icp_params.num_attempts; attempt++) {
        // Pick random initial condition inside of scene bounds
        Eigen::Affine3d est_tf_attempt = Eigen::Affine3d::Identity();
        for (int k = 0; k < 3; k++)
          est_tf_attempt.matrix()(k, 3) =
              modelCenter[k] + randrange(minPt[k], maxPt[k]);
        est_tf_attempt.matrix().block<3, 3>(0, 0) =
            Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();
        std::cout << "Starting condition: " << est_tf_attempt.matrix()
                  << std::endl;

        pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal,
                                   pcl::PointXYZRGBNormal>
            icp;
        icp.setTransformationEpsilon(1e-5);
        icp.setMaximumIterations(1);
        icp.setInputSource(object_cloud);
        icp.setInputTarget(complete_cloud_.makeShared());
        icp.setMaxCorrespondenceDistance(
            req.icp_params.initial_max_correspondence_distance);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr intermed(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::transformPointCloud(*object_cloud, *reg_result, est_tf_attempt);

        int iter = 0;
        double curr_error = std::numeric_limits<double>::infinity();
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr debug_cloud(
            new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        while (iter < req.icp_params.max_iterations) {
          intermed = reg_result;
          icp.setInputSource(intermed);
          icp.align(*reg_result);

          est_tf_attempt.matrix() =
              icp.getFinalTransformation().cast<double>() *
              est_tf_attempt.matrix();

          pcl::transformPointCloud(*object_cloud, *debug_cloud, est_tf_attempt);
          debug_cloud->header.stamp = 0;
          debug_cloud->header.frame_id = complete_cloud_.header.frame_id;
          debug_cloud_pub_.publish(debug_cloud);

          iter++;
        }

        std::cout << "Final fitness: " << icp.getFitnessScore() << std::endl;

        if (icp.getFitnessScore() < best_cost) {
          best_tf = est_tf_attempt;
          best_cost = icp.getFitnessScore();
        }
      }
    }

    data_update_lock_.unlock();
    return true;
  }

  void Update() {
    if (getUnixTime() - lastPublishTime_ > 0.1) {
      data_update_lock_.lock();
      complete_cloud_.header.stamp =
          0;  // Using RosTime causes 64->32 bit conversion errors
              // sometimes...
      pc2_pub_.publish(complete_cloud_.makeShared());
      data_update_lock_.unlock();
    }
  }
};

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "pointcloud_object_fitting_server",
            ros::init_options::AnonymousName);
  ros::NodeHandle n;

  ObjectFittingServer ofs(n);
  while (ros::ok()) {
    ofs.Update();
    ros::spinOnce();
    usleep(1000 * 1000);
  }
  return 0;
}