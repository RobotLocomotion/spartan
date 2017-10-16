#pragma once

#include <string>

#include "drake/multibody/rigid_body_tree.h"

class ReachabilityAnalyzer {
 public:
  ReachabilityAnalyzer(std::string yaml_config_path);
  void DoGraspAnalysis();

 private:
  struct GraspSearchEntry {
    Eigen::Vector3d grasp_direction;
    std::vector<double> vis_color;
  };
  std::map<std::string, GraspSearchEntry> grasp_search_entries_;

  void BuildGraspSearchOptions();
  void DoGraspSearch(std::string grasp_entry_name,
                     ReachabilityAnalyzer::GraspSearchEntry grasp_info);
  void DoGraspSearchPostProcessing();

  RigidBodyTree<double> model_;
  Eigen::VectorXd qnom_;
  std::shared_ptr<RigidBodyFrame<double>> end_effector_frame_;
  int link_index_;
  Eigen::Isometry3d frame_tf_;
  Eigen::Vector3d frame_grasp_direction_;

  std::default_random_engine e1_;

  Eigen::Vector3d min_val_;  // Lower corner of workspace
  Eigen::Vector3d max_val_;  // Upper corner of workspace
  Eigen::Vector3i steps_;    // # of steps spanning workspace
  Eigen::Vector3d step_size_;
  double position_tolerance_;
  double cone_tolerance_;

  bool save_output_;
  std::string output_directory_;

  std::vector<int> reachable_dirs_;
};