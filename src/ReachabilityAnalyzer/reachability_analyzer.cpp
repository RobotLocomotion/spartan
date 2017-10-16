#include <memory>
#include <string>
#include <vector>

#include "common_utils/math_utils.h"
#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

#include <gflags/gflags.h>
#include "spdlog/spdlog.h"

#include "RemoteTreeViewerWrapper.hpp"

#include "yaml-cpp/yaml.h"

DEFINE_string(config_filename, "config_filename",
              "YAML config file supplying search parameters.");

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Isometry3d;

using std::vector;
using std::string;

static int DoMain(void) {
  auto console = spdlog::stdout_color_mt("console");
  RemoteTreeViewerWrapper rm;
  auto model = std::make_unique<RigidBodyTree<double>>();
  YAML::Node config =
      YAML::LoadFile(expandEnvironmentVariables(FLAGS_config_filename));

  // Load configuration parameters from configuration file.
  if (config["urdf"]) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        expandEnvironmentVariables(config["urdf"].as<string>()),
        drake::multibody::joints::kFixed, model.get());
  } else {
    console->error("Config had no 'urdf' field!\n");
    exit(1);
  }

  // Iteration information to define the volume we'll search.
  if (!config["steps"] || !config["x_range"] || !config["y_range"] ||
      !config["z_range"]) {
    console->error(
        "Missing one or more of the fields ['steps', 'x_range', 'y_range', "
        "'z_range']!");
    exit(1);
  }

  Vector3i steps = StdVectorToEigenVector(config["steps"].as<vector<int>>());

  auto config_x_range = config["x_range"].as<vector<double>>();
  auto config_y_range = config["y_range"].as<vector<double>>();
  auto config_z_range = config["z_range"].as<vector<double>>();

  Vector3d min_val(config_x_range[0], config_y_range[0], config_z_range[0]);
  Vector3d max_val(config_x_range[1], config_y_range[1], config_z_range[1]);
  int n_pts = steps.prod();

  if (!config["position_tolerance"] || !config["cone_tolerance"]) {
    console->error(
        "Missing one or more of the fields ['position_tolerance', "
        "'cone_tolerance']!");
    exit(1);
  }
  const double position_tolerance = config["position_tolerance"].as<double>();
  const double cone_tolerance = config["cone_tolerance"].as<double>();

  // Search directions we wish to constrain the grasp -- i.e., all principal
  // directions.
  struct GraspSearchInfo {
    Vector3d grasp_direction;
    vector<double> vis_color;
  };
  std::map<string, GraspSearchInfo> grasp_search_options;
  grasp_search_options["+x"] = GraspSearchInfo(
      {.grasp_direction = Vector3d(1, 0, 0), .vis_color = {1.0, 0.0, 0.0}});
  grasp_search_options["+y"] = GraspSearchInfo(
      {.grasp_direction = Vector3d(0, 1, 0), .vis_color = {0.0, 1.0, 0.0}});
  grasp_search_options["+z"] = GraspSearchInfo(
      {.grasp_direction = Vector3d(0, 0, 1), .vis_color = {0.0, 0.0, 1.0}});
  grasp_search_options["-x"] = GraspSearchInfo(
      {.grasp_direction = Vector3d(-1, 0, 0), .vis_color = {1.0, 0.0, 1.0}});
  grasp_search_options["-y"] = GraspSearchInfo(
      {.grasp_direction = Vector3d(0, -1, 0), .vis_color = {0.0, 1.0, 1.0}});
  grasp_search_options["-z"] = GraspSearchInfo(
      {.grasp_direction = Vector3d(0, 0, -1), .vis_color = {1.0, 1.0, 0.0}});

  if (!config["frame_grasp_direction"]) {
    console->error("Missing the `frame_grasp_direction` field!");
    exit(1);
  }
  Vector3d frame_grasp_direction = StdVectorToEigenVector(
      config["frame_grasp_direction"].as<vector<double>>());

  // Extract the end effector body_index and position
  // info from the supplied frame name.
  if (!config["end_effector_frame_name"]) {
    console->error("Config has no 'end_effector_frame_name' field!");
    exit(1);
  }
  const auto end_effector_frame =
      model->findFrame(config["end_effector_frame_name"].as<string>());
  const int link_index = end_effector_frame->get_rigid_body().get_body_index();
  const Isometry3d frame_tf = end_effector_frame->get_transform_to_body();

  // Provide a dummy timespan for our single-time-point
  // constraints.
  const Vector2d tspan(0, 1);

  // The first pose that we test will be all-zeros.
  std::default_random_engine e1(0);
  VectorXd q0 = model->getRandomConfiguration(e1);
  VectorXd qnom = model->getZeroConfiguration();

  // Data structures for storing the position constraint
  // for the end effector, which is updated every loop.
  Vector3d pos_end;
  // Vector4d quat_end;
  Vector3d pos_lb, pos_ub;

  // Data structures that can be shared between ik calls.
  IKoptions ikoptions(model.get());
  // ikoptions.setMajorIterationsLimit(10);
  // ikoptions.setIterationsLimit(1000);
  console->info("Running with major iter lim {0:d} and iter limt {1:d}\n",
                ikoptions.getMajorIterationsLimit(),
                ikoptions.getIterationsLimit());

  VectorXd q_sol = qnom;
  int info = 0;
  std::vector<std::string> infeasible_constraint;

  // Non-penetration constraint
  AllBodiesClosestDistanceConstraint nonpen(
      model.get(), 0.00, std::numeric_limits<double>::infinity(), {}, {},
      tspan);

  // For calculating position from indices, precompute the step size
  // per index
  auto step_size =
      (max_val - min_val).array() /
      (steps.cast<double>().array() - 1).max(Vector3d::Ones().array());

  // # of unique directions reachable by the arm
  std::vector<int> reachable_dirs(n_pts, 0);

  for (const auto& it : grasp_search_options) {
    string grasp_dir_name = it.first;
    Vector3d grasp_dir = it.second.grasp_direction;
    vector<double> grasp_dir_color = it.second.vis_color;

    // Gaze constraint
    WorldGazeDirConstraint wgdc(model.get(), link_index,
                                frame_tf.rotation() * frame_grasp_direction,
                                grasp_dir, cone_tolerance, tspan);

    // Store volume information, temporarily, as a colored point cloud.
    Matrix3Xd pts(3, n_pts);
    std::vector<std::vector<double>> colors(n_pts, std::vector<double>(3));

    int k_tried = 0;
    int k_reachable = 0;
    int last_published_k_reachable = k_reachable - 1;
    console->info("With dir [{0:f},{1:f},{2:f}]:\n", grasp_dir[0], grasp_dir[1],
                  grasp_dir[2]);
    for (int x_i = 0; x_i < steps[0]; x_i++) {
      for (int y_i = 0; y_i < steps[1]; y_i++) {
        for (int z_i = 0; z_i < steps[2]; z_i++) {
          Vector3d pos_end =
              min_val +
              ((Vector3d(x_i, y_i, z_i).cast<double>().array()) * step_size)
                  .matrix();

          pos_lb = pos_end - Vector3d::Constant(position_tolerance);
          pos_ub = pos_end + Vector3d::Constant(position_tolerance);

          WorldPositionConstraint wpc(model.get(), link_index,
                                      frame_tf.translation(), pos_lb, pos_ub,
                                      tspan);

          const std::vector<const RigidBodyConstraint*> constraint_array{&wpc,
                                                                         &wgdc};

          // Important to randomly seed, because starting from the zero
          // configuration
          // confuses SNOPT sometimes (due to bad gradients at that posture).
          q0 = model->getRandomConfiguration(e1);
          inverseKin(model.get(), q0, qnom, constraint_array.size(),
                     constraint_array.data(), ikoptions, &q_sol, &info,
                     &infeasible_constraint);

          if (info == 1) {
            pts.col(k_reachable) = pos_end;
            colors[k_reachable][0] = 0.0;
            colors[k_reachable][1] = 1.0;
            colors[k_reachable][2] = 0.0;
            k_reachable++;
            reachable_dirs[k_tried]++;
          }

          k_tried++;
          // printf here because I'm abusing carriage returns and lack of
          // newlines...
          printf("\rTried %d/%d, reachable %d/%d", k_tried, n_pts, k_reachable,
                 n_pts);
          fflush(stdout);
          if (k_reachable % 100 == 0 &&
              k_reachable != last_published_k_reachable) {
            if (k_reachable > 0) {
              rm.publishPointCloud(pts.block(0, 0, 3, k_reachable),
                                   {"reachability", grasp_dir_name},
                                   {grasp_dir_color});
              rm.publishRigidBodyTree(*model, q_sol,
                                      Vector4d(0.5, 0.5, 0.5, 1.0),
                                      {"reachability", "robot"});
            }
            last_published_k_reachable = k_reachable;
          }
        }
      }
    }

    // printf here because I'm hacking a progress bar in...
    printf("\rTried %d/%d, reachable %d/%d\n", k_tried, n_pts, k_reachable,
           n_pts);
    if (k_reachable > 0) {
      rm.publishPointCloud(pts.block(0, 0, 3, k_reachable),
                           {"reachability", grasp_dir_name}, {grasp_dir_color});
      rm.publishRigidBodyTree(*model, q_sol, Vector4d(0.5, 0.5, 0.5, 1.0),
                              {"reachability", "robot"});
    }
  }

  // And collate and publish a complete dextrous-workspace color-mapped point
  // cloud
  Matrix3Xd all_pts(3, n_pts);
  std::vector<std::vector<double>> dextrous_colors(n_pts,
                                                   std::vector<double>(4, 0.0));
  int k = 0;
  for (int x_i = 0; x_i < steps[0]; x_i++) {
    for (int y_i = 0; y_i < steps[1]; y_i++) {
      for (int z_i = 0; z_i < steps[2]; z_i++) {
        Vector3d pos_end =
            min_val +
            ((Vector3d(x_i, y_i, z_i).cast<double>().array()) * step_size)
                .matrix();
        all_pts.col(k) = pos_end;

        double good_fraction =
            ((double)reachable_dirs[k]) / ((double)grasp_search_options.size());
        dextrous_colors[k] = {1. - good_fraction, good_fraction,
                              1. - (fabs(good_fraction - 0.5) * 2),
                              good_fraction};
        k++;
      }
    }
  }

  rm.publishPointCloud(all_pts, {"reachability", "manipulable workspace"},
                       dextrous_colors);

  if (config["output_directory"]){
    string output_directory = expandEnvironmentVariables(config["output_directory"].as<string>());
    const int dir_err = system((string("mkdir -p ") + output_directory).c_str());
    if (dir_err){
      console->error("Error creating output directory {0}", output_directory);
      exit(1);
    }
    auto all_pts_vtk = PolyDataFromMatrix3Xd(all_pts);
    AddColorToPolyData(all_pts_vtk, dextrous_colors);
    WritePolyData(all_pts_vtk, (output_directory + "/all_pts.vtp").c_str());
  }

  return 0;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return DoMain();
}