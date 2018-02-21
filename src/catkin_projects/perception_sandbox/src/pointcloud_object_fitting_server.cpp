#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <unistd.h>
#include <iostream>
#include <mutex>
#include <string>

#include <Eigen/Geometry>
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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
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
#include "common_utils/pcl_vtk_utils.h"
#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"

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

DEFINE_bool(run_test, false, "Run in test mode?");
DEFINE_string(test_scene, "last_scene.pcd", "Scene to test on");
DEFINE_string(test_scene_frame_id, "base", "Scene's frame id");
DEFINE_string(test_object, "none.obj", "Object to test on");
DEFINE_double(test_object_scale, 1.0, "Object's scaling'");

using namespace std;

bool myRegionGrowing(const pcl::PointXYZRGBNormal& point_a,
                     const pcl::PointXYZRGBNormal& point_b,
                     float squared_distance) {
  printf("Here\n");
  Eigen::Map<const Eigen::Vector3f> point_a_normal =
                                        point_a.getNormalVector3fMap(),
                                    point_b_normal =
                                        point_b.getNormalVector3fMap();
  std::cout << "d " << squared_distance << ", " << point_a_normal << ", "
            << point_b_normal << std::endl;
  exit(1);
  if (squared_distance > powf(0.02, 2.0)) return false;
  if (fabs(point_a_normal.dot(point_b_normal)) < 0.5 &&
      squared_distance > powf(0.02, 2.0))
    return false;
  return true;
}

class ObjectFittingServer {
  ros::NodeHandle nh_;

  ros::Publisher pc2_pub_;
  double lastPublishTime_;
  ros::Publisher debug_cloud_pub_;
  ros::Publisher partial_cloud_pub_;
  ros::Publisher segmented_cloud_pub_;

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
        partial_cloud_pub_(
            nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>>(
                "/object_fitting/partial_cloud", 1)),
        segmented_cloud_pub_(
            nh_.advertise<pcl::PointCloud<pcl::PointXYZRGBNormal>>(
                "/object_fitting/segmented_cloud", 1)),
        lastPublishTime_(getUnixTime() - 100.),
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

  void SetPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_cloud) {
    data_update_lock_.lock();
    complete_cloud_ = *scene_cloud;
    data_update_lock_.unlock();
  }

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
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(complete_cloud_);

    pcl::io::savePCDFileASCII("last_scene.pcd", complete_cloud_);

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

  std::vector<pcl::PointIndices> OversegmentPointCloud(
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
    std::vector<pcl::PointIndices> clusters;

    // Pick 100 random points and take all points within a set
    // distance
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIndices(100);
    std::vector<float> pointDistances(100);
    for (int i = 0; i < 100; i++) {
      const auto& searchPt = cloud->points[rand() % cloud->size()];
      if (kdtree.radiusSearch(searchPt, 0.05, pointIndices, pointDistances) >
          0) {
        pcl::PointIndices new_cluster;
        new_cluster.indices = pointIndices;
        clusters.push_back(new_cluster);
      }
    }

    // Not perfect because clusters may overlap, but meh
    pcl::PointCloud<pcl::PointXYZRGBNormal> colored_segment_cloud;
    colored_segment_cloud += *cloud;
    for (int i = 0; i < clusters.size(); ++i) {
      char r = rand() % 255;
      char g = rand() % 255;
      char b = rand() % 255;
      for (int j = 0; j < clusters[i].indices.size(); ++j) {
        colored_segment_cloud.points[clusters[i].indices[j]].r = r;
        colored_segment_cloud.points[clusters[i].indices[j]].g = g;
        colored_segment_cloud.points[clusters[i].indices[j]].b = b;
      }
    }

    colored_segment_cloud.header.stamp = 0;
    colored_segment_cloud.header.frame_id = cloud->header.frame_id;
    segmented_cloud_pub_.publish(colored_segment_cloud);

    // Also save them to "clusters" dir in PCD
    struct stat st = {0};
    if (stat("clusters", &st) == -1) {
      mkdir("clusters", 0700);
    }
    for (int i = 0; i < clusters.size(); ++i) {
      char namebuf[100];
      sprintf(namebuf, "clusters/cluster_%03d.pcd", i);
      pcl::PointCloud<pcl::PointXYZRGBNormal> cluster_cloud;
      for (int j = 0; j < clusters[i].indices.size(); ++j) {
        cluster_cloud.push_back(cloud->points[clusters[i].indices[j]]);
      }
      pcl::io::savePCDFileBinary(namebuf, cluster_cloud);
    }
    return clusters;
  }

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr SubtractSceneCloud(
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr subtract) {
    // Build scene cloud that doesn't include points near
    // the currently fit objects.

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reduced_scene_cloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    if (subtract->size() == 0) {
      *reduced_scene_cloud += *cloud;
    } else {
      printf("Pruning scene cloud against %lu fit points...\n",
             subtract->size());
      pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      kdtree.setInputCloud(subtract);
      for (const auto& searchPt : cloud->points) {
        if (kdtree.nearestKSearch(searchPt, 1, pointIdxNKNSearch,
                                  pointNKNSquaredDistance) > 0) {
          if (pointNKNSquaredDistance[0] > powf(0.02, 2.)) {  // Prune distance
            reduced_scene_cloud->push_back(searchPt);
          }
        }
      }
    }
    reduced_scene_cloud->header.stamp = 0;
    reduced_scene_cloud->header.frame_id = cloud->header.frame_id;
    partial_cloud_pub_.publish(reduced_scene_cloud);
    return reduced_scene_cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr LoadAndResampleMeshFromPath(
      const std::string& path, double scale = 1.0) {
    printf("Loading msg from %s\n", path.c_str());
    vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(path.c_str());
    cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from "
         << path << endl;
    cloudPolyData = DownsampleAndCleanPolyData(cloudPolyData, 0.01 / scale);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr object_cloud =
        PointCloudFromPolyDataWithNormals<pcl::PointXYZRGBNormal>(
            cloudPolyData);

    // Scale as requested
    if (scale != 1.0) {
      Eigen::Affine3d tf = Eigen::Affine3d::Identity();
      tf.matrix().block<3, 3>(0, 0) *= scale;

      pcl::transformPointCloud(*object_cloud, *object_cloud, tf);
    }

    return object_cloud;
  }

  bool FitObjectsByIcp(perception_sandbox::FitObjectsByIcp::Request& req,
                       perception_sandbox::FitObjectsByIcp::Response& res) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr complete_cloud_local(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Copy the scene cloud locally so we don't have to hold this
    // lock open.
    data_update_lock_.lock();
    *complete_cloud_local = complete_cloud_;
    data_update_lock_.unlock();

    // [Over]segment it
    auto segments = OversegmentPointCloud(complete_cloud_local);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr full_fit_objects_cloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // For each mesh...
    int n_objects = req.mesh_paths.size();
    for (int i = 0; i < n_objects; i++) {
      // Fit it against the points we haven't dealt with yet.
      auto reduced_scene_cloud =
          SubtractSceneCloud(complete_cloud_local, full_fit_objects_cloud);

      double best_cost = std::numeric_limits<double>::infinity();
      Eigen::Affine3d best_tf = Eigen::Affine3d::Identity();
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr debug_cloud(
          new pcl::PointCloud<pcl::PointXYZRGBNormal>);

      if (reduced_scene_cloud->points.size() < 10) {
        printf(
            "Not enough points in reduced scene cloud: have only %lu after "
            "pruning\n",
            reduced_scene_cloud->points.size());
      } else {
        // Load and resample the specified object
        auto object_cloud =
            LoadAndResampleMeshFromPath(req.mesh_paths[i].data, req.scales[i]);

        // Randomly seed transform at random reasonable locations,
        // and then refine with ICP. Take the best one in terms of
        // ICP residual error.
        pcl::PointXYZRGBNormal minPtPcl, maxPtPcl;
        pcl::getMinMax3D(*reduced_scene_cloud, minPtPcl, maxPtPcl);
        Eigen::Vector3d minPt = Vector3dFromPclPoint(minPtPcl);
        Eigen::Vector3d maxPt = Vector3dFromPclPoint(maxPtPcl);

        pcl::PointXYZRGBNormal modelMinPtPcl, modelMaxPtPcl;
        pcl::getMinMax3D(*object_cloud, modelMinPtPcl, modelMaxPtPcl);
        Eigen::Vector3d modelCenter = (Vector3dFromPclPoint(modelMinPtPcl) +
                                       Vector3dFromPclPoint(modelMaxPtPcl)) /
                                      2.;

        for (int attempt = 0; attempt < req.icp_params.num_attempts;
             attempt++) {
          // Pick random initial condition inside of scene bounds
          Eigen::Affine3d est_tf_attempt = Eigen::Affine3d::Identity();
          Eigen::Affine3d est_tf_last;
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
          icp.setInputTarget(reduced_scene_cloud);
          icp.setMaxCorrespondenceDistance(
              req.icp_params.initial_max_correspondence_distance);

          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result(
              new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr intermed(
              new pcl::PointCloud<pcl::PointXYZRGBNormal>);
          pcl::transformPointCloud(*object_cloud, *reg_result, est_tf_attempt);

          int iter = 0;
          double corresp_dist =
              req.icp_params.initial_max_correspondence_distance;
          double curr_error = std::numeric_limits<double>::infinity();

          while (iter < req.icp_params.max_iterations) {
            intermed = reg_result;
            icp.setInputSource(intermed);
            icp.align(*reg_result);

            est_tf_last = est_tf_attempt;
            est_tf_attempt.matrix() =
                icp.getFinalTransformation().cast<double>() *
                est_tf_attempt.matrix();
            double diff =
                (est_tf_last.matrix() - est_tf_attempt.matrix()).norm();
            if (diff < 0.001) {
              break;
            }

            corresp_dist = fmax(0.01, corresp_dist * 0.95);
            icp.setMaxCorrespondenceDistance(corresp_dist);

            if (iter % 50 == 0) {
              pcl::transformPointCloud(*object_cloud, *debug_cloud,
                                       est_tf_attempt);
              debug_cloud->header.stamp = 0;
              debug_cloud->header.frame_id =
                  complete_cloud_local->header.frame_id;
              debug_cloud_pub_.publish(debug_cloud);
            }
            iter++;
          }

          std::cout << "Final fitness after " << iter
                    << " iters: " << icp.getFitnessScore() << std::endl;

          if (icp.getFitnessScore() < best_cost) {
            best_tf = est_tf_attempt;
            best_cost = icp.getFitnessScore();
          }
        }

        pcl::transformPointCloud(*object_cloud, *debug_cloud, best_tf);
        *full_fit_objects_cloud += *debug_cloud;
      }

      std::cout << "Best fitness ever: " << best_tf.matrix() << " with fitness "
                << best_cost << std::endl;
      geometry_msgs::Transform tf_out;
      tf::transformEigenToMsg(best_tf, tf_out);
      res.poses.push_back(tf_out);
    }

    // render all fits combined
    full_fit_objects_cloud->header.stamp = 0;
    full_fit_objects_cloud->header.frame_id =
        complete_cloud_local->header.frame_id;
    debug_cloud_pub_.publish(full_fit_objects_cloud);

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

  if (FLAGS_run_test) {
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_cloud_loaded(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(
            FLAGS_test_scene, *scene_cloud_loaded) == -1)  //* load the file
    {
      printf("Couldn't load test cloud.\n");
      return -1;
    }

    scene_cloud_loaded->header.frame_id = FLAGS_test_scene_frame_id;

    ofs.SetPointCloud(scene_cloud_loaded);
    usleep(1000 * 200);
    ofs.Update();

    // Generate an ICP request message
    perception_sandbox::FitObjectsByIcp::Request fake_req;
    perception_sandbox::FitObjectsByIcp::Response fake_resp;

    fake_req.mesh_paths.push_back(std_msgs::String());
    fake_req.mesh_paths[0].data = string(FLAGS_test_object);
    fake_req.scales.push_back(FLAGS_test_object_scale);
    fake_req.icp_params.num_attempts = 1;
    fake_req.icp_params.initial_max_correspondence_distance = 0.1;
    fake_req.icp_params.max_iterations = 5;

    ofs.FitObjectsByIcp(fake_req, fake_resp);

  } else {
    while (ros::ok()) {
      ofs.Update();
      ros::spinOnce();
    }
  }

  return 0;
}