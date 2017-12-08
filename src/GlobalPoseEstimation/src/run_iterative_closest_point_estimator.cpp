/*
 */

#include "common_utils/math_utils.h"
#include "common_utils/pcl_utils.h"
#include "common_utils/pcl_vtk_utils.h"
#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include "RemoteTreeViewerWrapper.hpp"

#include "yaml-cpp/yaml.h"

#include "iterative_closest_point.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

typedef pcl::PointXYZ PointType;

double run_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud,
               Eigen::Affine3d* est_tf, int max_iterations,
               double initial_max_correspondence_distance,
               double min_correspondence_distance, bool visualize = false) {
  RemoteTreeViewerWrapper rm;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(1e-5);
  icp.setMaximumIterations(1);
  icp.setInputSource(model_cloud);
  icp.setInputTarget(scene_cloud);
  icp.setMaxCorrespondenceDistance(initial_max_correspondence_distance);

  // For vis
  Matrix3Xd model_pts =
      convertPclPointXyzEtcToMatrix3Xd<pcl::PointXYZ>(model_cloud);
  rm.publishPointCloud((*est_tf) * model_pts, {"model_pts_tf"});

  pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr intermed(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*model_cloud, *reg_result, est_tf->matrix());

  int iter = 0;
  double curr_error = std::numeric_limits<double>::infinity();

  while (iter < max_iterations) {
    intermed = reg_result;
    icp.setInputSource(intermed);
    icp.align(*reg_result);

    est_tf->matrix() =
        icp.getFinalTransformation().cast<double>() * est_tf->matrix();

    if (visualize) {
      rm.publishPointCloud((*est_tf) * model_pts, {"model_pts_tf"},
                           {{1.0, 0.1, 0.1}});
      usleep(1000 * 10);
    }

    double new_error = icp.getFitnessScore();
    if (fabs(curr_error - new_error) < 1e-5) {
      icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() *
                                       0.95);
    }
    curr_error = new_error;

    if (icp.getMaxCorrespondenceDistance() < min_correspondence_distance) {
      break;
    }

    iter++;
  }

  return icp.getFitnessScore();
}

bool sortbyfirst(const pair<double, Affine3d>& a,
                 const pair<double, Affine3d>& b) {
  return (a.first < b.first);
}

int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc < 4) {
    printf(
        "Use: run_iterative_closest_point_estimator <scene cloud> <model "
        "cloud> <config file> <output file>\n");
    exit(-1);
  }

  time_t _tm = time(NULL);
  struct tm* curtime = localtime(&_tm);

  // Bring in config file
  string scene_cloud_filename = string(argv[1]);
  string model_cloud_filename = string(argv[2]);
  string config_filename = string(argv[3]);

  // Load in config
  YAML::Node config = YAML::LoadFile(config_filename);

  if (config["detector_options"] == NULL) {
    runtime_error("Need detector options.");
  }

  auto random_icp_config = config["detector_options"];
  double downsample_spacing = -1;
  if (random_icp_config["downsample_spacing"]) {
    downsample_spacing = random_icp_config["downsample_spacing"].as<double>();
  }
  int num_samples = 100;
  if (random_icp_config["num_samples"]) {
    num_samples = random_icp_config["num_samples"].as<int>();
  }
  bool visualize_icp = false;
  if (random_icp_config["visualize_icp"]) {
    visualize_icp = random_icp_config["visualize_icp"].as<bool>();
  }

  // Load point cloud
  Matrix3Xd scene_pts =
      LoadAndDownsamplePolyData(scene_cloud_filename, downsample_spacing);
  // Load model cloud
  Matrix3Xd model_pts =
      LoadAndDownsamplePolyData(model_cloud_filename, downsample_spacing);

  auto scene_pts_pcl =
      convertMatrix3XdToPclPointXyzEtc<pcl::PointXYZ>(scene_pts);
  auto model_pts_pcl =
      convertMatrix3XdToPclPointXyzEtc<pcl::PointXYZ>(model_pts);

  // Visualize the results using the drake visualizer.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts, {"scene_pts_loaded"}, {{0.0, 1.0, 0.0}});
  rm.publishPointCloud(model_pts, {"model_pts_loaded"}, {{0.0, 0.0, 1.0}});

  // Scene cloud total width
  Vector3d scene_max = scene_pts.rowwise().maxCoeff();
  Vector3d scene_min = scene_pts.rowwise().minCoeff();

  vector<pair<double, Affine3d>> est_tfs(
      num_samples,
      pair<double, Affine3d>(std::numeric_limits<double>::infinity(),
                             Affine3d::Identity()));

  for (int i = 0; i < est_tfs.size(); i++) {
    Affine3d* est_tf = &est_tfs[i].second;

    // Start from random reasonable guess
    // Center on model...
    est_tf->translation() = -model_pts.rowwise().mean();
    // and transform randomly in the scene bounding box
    for (int k = 0; k < 3; k++) {
      est_tf->translation()[k] = randrange(scene_min[k], scene_max[k]);
    }
    // And transform to random rotation
    est_tf->matrix().block<3, 3>(0, 0) =
        Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();

    est_tfs[i].first = run_icp(scene_pts_pcl, model_pts_pcl, est_tf, 1000, 0.05,
                               0.01, visualize_icp);

    /*
    char buf[100];
    sprintf(buf, "%f", est_tfs[i].first);
    rm.publishPointCloud((*est_tf) * model_pts,
                         {"model_pts_tf", {"possible_sol"}, string(buf)},
                         {{0.1, 0.1, 1.0}});
                         */
  }

  sort(est_tfs.begin(), est_tfs.end(), sortbyfirst);
  rm.publishPointCloud((est_tfs[0].second) * model_pts, {"model_pts_tf_best"},
                       {{1.0, 0.1, 0.5}});

  if (argc > 3) {
    string output_file = string(argv[4]);
    YAML::Emitter out;
    out << YAML::BeginMap;
    {
      out << YAML::Key << "scene";
      out << YAML::Value << scene_cloud_filename;

      out << YAML::Key << "config";
      out << YAML::Value << config_filename;

      out << YAML::Key << "solutions";
      out << YAML::BeginSeq;
      {
        for (const auto& est_tf_pair : est_tfs) {
          Affine3d est_tf = est_tf_pair.second;

          VectorXd q_out(7);
          q_out.block<3, 1>(0, 0) = est_tf.translation();
          q_out.block<4, 1>(3, 0) =
              drake::math::rotmat2quat(est_tf.rotation());
          out << YAML::BeginMap;
          {
            out << YAML::Key << "models";
            out << YAML::Value << YAML::BeginSeq;
            {
              out << YAML::BeginMap;
              {
                out << YAML::Key << "model";
                out << YAML::Value << model_cloud_filename;
                out << YAML::Key << "q";
                out << YAML::Value << YAML::Flow
                    << vector<double>(q_out.data(),
                                      q_out.data() + q_out.rows());
              }
              out << YAML::EndMap;
            }
            out << YAML::EndSeq;

            out << YAML::Key << "history";
            out << YAML::Value << YAML::BeginMap;
            {
              out << YAML::Key << "objective";
              out << YAML::Value << est_tf_pair.first;
            }
            out << YAML::EndMap;
          }
          out << YAML::EndMap;
        }
      }
      out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    ofstream fout(output_file);
    fout << out.c_str();
    fout.close();
  }

  return 0;
}
