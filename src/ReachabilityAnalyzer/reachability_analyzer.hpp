#pragma once

#include <string>

#include "drake/multibody/rigid_body_tree.h"

class ReachabilityAnalyzer {
 public:
  ReachabilityAnalyzer(std::string yaml_config_path);

  // This method calls each stage of the analysis pipeline
  // in sequence:
  //   1) BuildReachabilitySearchOptions, which
  //      populates reachability_search_entries.
  //   2) Calls DoReachabilitySearch for each unique entry
  //      in the reachability search entries.
  //   3) Calls DoReachabilitySearchPostProcessing to build
  //      the manipulable workspace.
  void DoReachabilityAnalysis();

 private:
  struct ReachabilitySearchEntry {
    Eigen::Vector3d grasp_direction;
    std::vector<double> vis_color;
  };
  std::map<std::string, ReachabilitySearchEntry> reachability_search_entries_;

  void BuildReachabilitySearchOptions();

  void DoReachabilitySearch(
      std::string reachability_entry_name,
      ReachabilityAnalyzer::ReachabilitySearchEntry reachability_info);
  void DoReachabilitySearchPostProcessing();

  // Model we'll do search over, and some info about it
  RigidBodyTree<double> model_;
  Eigen::VectorXd qnom_;
  std::shared_ptr<RigidBodyFrame<double>> end_effector_frame_;
  int link_index_;
  Eigen::Isometry3d frame_tf_;
  Eigen::Vector3d frame_grasp_direction_;

  // RNG we'll use to seed the IK
  std::default_random_engine e1_;

  // Tolerances for IK
  double position_tolerance_;
  double cone_tolerance_;

  // Workspace boundary definitions
  Eigen::Vector3d min_val_;  // Lower corner of workspace
  Eigen::Vector3d max_val_;  // Upper corner of workspace
  Eigen::Vector3i steps_;    // # of steps spanning workspace
  Eigen::Vector3d step_size_;

  // Manipulability analysis: the nth bin here indicates that the nth bin
  // (indexed as x_i * steps_[1] * steps_[2] + y_i * steps_[2] + z_i)
  // was reachable from M different directions
  std::vector<int> reachable_dirs_;

  // Where (and whether) we'll save our products as VTP files
  bool save_output_;
  std::string output_directory_;
};