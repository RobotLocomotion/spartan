#include <memory>
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

#include "reachability_analyzer.hpp"

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

typedef vector<vector<double>> ColorVector;
// Save a colorized point cloud as a VTP file
void SaveColorizedVTP(const Matrix3Xd& pts, const ColorVector& colors,
                      const char* filename) {
  auto pts_vtk = PolyDataFromMatrix3Xd(pts);
  AddColorToPolyData(pts_vtk, colors);
  WritePolyData(pts_vtk, filename);
}

ReachabilityAnalyzer::ReachabilityAnalyzer(std::string yaml_config_path)
    : e1_(0) {
  auto console = spdlog::get("console");

  YAML::Node config = YAML::LoadFile(yaml_config_path);

  if (config["urdf"]) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        expandEnvironmentVariables(config["urdf"].as<string>()),
        drake::multibody::joints::kFixed, &model_);
  } else {
    console->error("Config had no 'urdf' field!\n");
    exit(1);
  }
  qnom_ = model_.getZeroConfiguration();

  RemoteTreeViewerWrapper rm;
  rm.publishRigidBodyTree(model_, qnom_, Vector4d(0.5, 0.5, 0.5, 1.0),
                          {"reachability", "robot"});

  // Iteration information to define the volume we'll search.
  if (!config["steps"] || !config["x_range"] || !config["y_range"] ||
      !config["z_range"]) {
    console->error(
        "Missing one or more of the fields ['steps', 'x_range', 'y_range', "
        "'z_range']!");
    exit(1);
  }

  steps_ = StdVectorToEigenVector(config["steps"].as<vector<int>>());

  auto config_x_range = config["x_range"].as<vector<double>>();
  auto config_y_range = config["y_range"].as<vector<double>>();
  auto config_z_range = config["z_range"].as<vector<double>>();

  min_val_ = Vector3d(config_x_range[0], config_y_range[0], config_z_range[0]);
  max_val_ = Vector3d(config_x_range[1], config_y_range[1], config_z_range[1]);

  // For calculating position from indices, precompute the step size
  // per index
  step_size_ =
      (max_val_ - min_val_).array() /
      (steps_.cast<double>().array() - 1).max(Vector3d::Ones().array());

  if (!config["position_tolerance"] || !config["cone_tolerance"]) {
    console->error(
        "Missing one or more of the fields ['position_tolerance', "
        "'cone_tolerance']!");
    exit(1);
  }
  position_tolerance_ = config["position_tolerance"].as<double>();
  cone_tolerance_ = config["cone_tolerance"].as<double>();

  if (!config["frame_grasp_direction"]) {
    console->error("Missing the `frame_grasp_direction` field!");
    exit(1);
  }
  frame_grasp_direction_ = StdVectorToEigenVector(
      config["frame_grasp_direction"].as<vector<double>>());

  // Extract the end effector body_index and position
  // info from the supplied frame name.
  if (!config["end_effector_frame_name"]) {
    console->error("Config has no 'end_effector_frame_name' field!");
    exit(1);
  }
  end_effector_frame_ =
      model_.findFrame(config["end_effector_frame_name"].as<string>());
  link_index_ = end_effector_frame_->get_rigid_body().get_body_index();
  frame_tf_ = end_effector_frame_->get_transform_to_body();

  if (config["output_directory"]) {
    output_directory_ =
        expandEnvironmentVariables(config["output_directory"].as<string>());
    const int dir_err =
        system((string("mkdir -p ") + output_directory_).c_str());
    if (dir_err) {
      console->error("Error creating output directory {0}", output_directory_);
      exit(1);
    }
    save_output_ = true;
  } else {
    output_directory_ = "";
    save_output_ = false;
  }
};

// Options unique to a single
// grasp direction-constrained
// scan over the workspace.
void ReachabilityAnalyzer::BuildReachabilitySearchOptions() {
  reachability_search_entries_.clear();
  reachability_search_entries_["+x"] = ReachabilityAnalyzer::ReachabilitySearchEntry(
      {.grasp_direction = Vector3d(1, 0, 0), .vis_color = {1.0, 0.0, 0.0}});
  reachability_search_entries_["+y"] = ReachabilityAnalyzer::ReachabilitySearchEntry(
      {.grasp_direction = Vector3d(0, 1, 0), .vis_color = {0.0, 1.0, 0.0}});
  reachability_search_entries_["+z"] = ReachabilityAnalyzer::ReachabilitySearchEntry(
      {.grasp_direction = Vector3d(0, 0, 1), .vis_color = {0.0, 0.0, 1.0}});
  reachability_search_entries_["-x"] = ReachabilityAnalyzer::ReachabilitySearchEntry(
      {.grasp_direction = Vector3d(-1, 0, 0), .vis_color = {1.0, 0.0, 1.0}});
  reachability_search_entries_["-y"] = ReachabilityAnalyzer::ReachabilitySearchEntry(
      {.grasp_direction = Vector3d(0, -1, 0), .vis_color = {0.0, 1.0, 1.0}});
  reachability_search_entries_["-z"] = ReachabilityAnalyzer::ReachabilitySearchEntry(
      {.grasp_direction = Vector3d(0, 0, -1), .vis_color = {1.0, 1.0, 0.0}});
}

void ReachabilityAnalyzer::DoReachabilitySearch(
    string reachability_entry_name,
    ReachabilityAnalyzer::ReachabilitySearchEntry reachability_info) {
  auto console = spdlog::get("console");

  RemoteTreeViewerWrapper rm;
  Vector3d grasp_dir = reachability_info.grasp_direction;
  vector<double> grasp_dir_color = reachability_info.vis_color;

  // Provide a dummy timespan for our single-time-point
  // constraints.
  const Vector2d tspan(0, 1);

  // Data structures that are reused every loop.
  Vector3d pos_end;
  Vector3d pos_lb, pos_ub;

  IKoptions ikoptions(&model_);
  // iksetMajorIterationsLimit(10);
  // iksetIterationsLimit(1000);

  VectorXd q_sol(qnom_.size());
  VectorXd q0;
  int info = 0;
  vector<std::string> infeasible_constraint;

  // Non-penetration constraint
  AllBodiesClosestDistanceConstraint nonpen(
      &model_, 0.00, std::numeric_limits<double>::infinity(), {}, {}, tspan);

  // Gaze constraint
  WorldGazeDirConstraint wgdc(&model_, link_index_,
                              frame_tf_.rotation() * frame_grasp_direction_,
                              grasp_dir, cone_tolerance_, tspan);

  // Store volume information as a colored point cloud.
  int n_pts = steps_.prod();
  Matrix3Xd pts(3, n_pts);
  ColorVector colors(n_pts, vector<double>(4, 0.));

  // These constants provide state for telling how far through the search
  // we are, and for occasionally publishing status and visualization.
  int k_tried = 0;
  int k_reachable = 0;
  int last_published_k_reachable = k_reachable - 1;

  for (int x_i = 0; x_i < steps_[0]; x_i++) {
    for (int y_i = 0; y_i < steps_[1]; y_i++) {
      for (int z_i = 0; z_i < steps_[2]; z_i++) {
        Vector3d pos_end = min_val_ +
                           ((Vector3d(x_i, y_i, z_i).cast<double>().array()) *
                            step_size_.array())
                               .matrix();

        pos_lb = pos_end - Vector3d::Constant(position_tolerance_);
        pos_ub = pos_end + Vector3d::Constant(position_tolerance_);

        WorldPositionConstraint wpc(&model_, link_index_,
                                    frame_tf_.translation(), pos_lb, pos_ub,
                                    tspan);

        const vector<const RigidBodyConstraint*> constraint_array{&wpc, &wgdc};

        // Important to randomly seed, because starting from the zero
        // configuration
        // confuses SNOPT sometimes (due to bad gradients at that posture).
        q0 = model_.getRandomConfiguration(e1_);
        inverseKin(&model_, q0, qnom_, constraint_array.size(),
                   constraint_array.data(), ikoptions, &q_sol, &info,
                   &infeasible_constraint);

        if (info == 1) {
          pts.col(k_reachable) = pos_end;
          colors[k_reachable][0] = 0.0;
          colors[k_reachable][1] = 1.0;
          colors[k_reachable][2] = 0.0;
          colors[k_reachable][3] = 1.0;
          k_reachable++;
          reachable_dirs_[k_tried] += 1;
        }

        k_tried++;
        // printf here because I'm abusing carriage returns and lack of
        // newlines...
        printf("\rTried %d/%d, reachable %d/%d", k_tried, n_pts, k_reachable,
               n_pts);
        fflush(stdout);
        if (k_reachable - last_published_k_reachable >= 50 &&
            k_reachable != last_published_k_reachable) {
          rm.publishPointCloud(pts.block(0, 0, 3, k_reachable),
                               {"reachability", reachability_entry_name},
                               {grasp_dir_color});

          if (info == 1) {
            rm.publishRigidBodyTree(model_, q_sol, Vector4d(0.5, 0.5, 0.5, 1.0),
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
                         {"reachability", reachability_entry_name}, {grasp_dir_color});
  }

  if (save_output_) {
    SaveColorizedVTP(
        pts.block(0, 0, 3, k_reachable), colors,
        (output_directory_ + "/" + reachability_entry_name + ".vtp").c_str());
  }
}

void ReachabilityAnalyzer::DoReachabilitySearchPostProcessing() {
  auto console = spdlog::get("console");

  RemoteTreeViewerWrapper rm;
  int n_pts = steps_.prod();

  Matrix3Xd all_pts(3, n_pts);
  ColorVector dextrous_colors(n_pts, vector<double>(4, 0.0));
  int k = 0;
  for (int x_i = 0; x_i < steps_[0]; x_i++) {
    for (int y_i = 0; y_i < steps_[1]; y_i++) {
      for (int z_i = 0; z_i < steps_[2]; z_i++) {
        Vector3d pos_end = min_val_ +
                           ((Vector3d(x_i, y_i, z_i).cast<double>().array()) *
                            step_size_.array())
                               .matrix();
        all_pts.col(k) = pos_end;

        double good_fraction = ((double)reachable_dirs_[k]) /
                               ((double)reachability_search_entries_.size());
        dextrous_colors[k] = {1. - good_fraction, good_fraction,
                              1. - (fabs(good_fraction - 0.5) * 2),
                              good_fraction};
        k++;
      }
    }
  }

  rm.publishPointCloud(all_pts, {"reachability", "manipulable workspace"},
                       dextrous_colors);

  if (save_output_) {
    SaveColorizedVTP(all_pts, dextrous_colors,
                     (output_directory_ + "/manipulability.vtp").c_str());
  }
}

void ReachabilityAnalyzer::DoReachabilityAnalysis() {
  // Build grasp search directions we wish to constrain the grasp -- i.e., all
  // principal
  // directions.
  BuildReachabilitySearchOptions();

  // # of unique directions reachable by the arm -- indexed into
  // in same order as the search performed in DoReachabilitySearch.
  // TODO(gizatt) Replace by EigenNdArray once I PR that tool.
  reachable_dirs_.clear();
  reachable_dirs_.resize(steps_.prod(), 0);

  for (const auto& it : reachability_search_entries_) {
    DoReachabilitySearch(it.first, it.second);
  }

  // And collate and publish a complete dextrous-workspace color-mapped point
  // cloud.
  DoReachabilitySearchPostProcessing();
}

static int DoMain(void) {
  RemoteTreeViewerWrapper rm;

  ReachabilityAnalyzer analyzer(
      expandEnvironmentVariables(FLAGS_config_filename));

  analyzer.DoReachabilityAnalysis();

  return 0;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return DoMain();
}