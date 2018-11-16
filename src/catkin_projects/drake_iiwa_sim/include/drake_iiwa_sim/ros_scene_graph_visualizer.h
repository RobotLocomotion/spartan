#pragma once

/// @file Given a scene graph, takes as input the
/// state of everything in it, and visualizes them
/// using ROS interactive markers so they're
/// visible in RViz.

#include "drake/common/drake_deprecated.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/scene_graph.h"

#include "ros/ros.h"
#include <interactive_markers/interactive_marker_server.h>


namespace drake_iiwa_sim {

class RosSceneGraphVisualizer : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosSceneGraphVisualizer)

  /// Contains a SimpleActionServer server spoofing the
  /// real WSG50 driver used in Spartan.

  /// @param initial_position the commanded position to output if no command
  /// message has been received yet.
  ///
  /// @param initial_force the commanded force limit to output if no command
  /// message has been received yet.
  RosSceneGraphVisualizer(const drake::geometry::SceneGraph<double>& scene_graph,
                          std::string server_name = "scene_graph",
                          double draw_period=0.033333);


  const drake::systems::InputPort<double>& get_pose_bundle_input_port() const {
    return this->get_input_port(pose_bundle_input_port_);
  }

 protected:
  std::string MakeFullName(const std::string& input_name, int robot_num) const;
  void DoInitialization() const;
  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::PublishEvent<double>*>& event) const override;

 private:
  const drake::systems::InputPortIndex pose_bundle_input_port_{};
  const drake::geometry::SceneGraph<double>& scene_graph_{};
  mutable ros::NodeHandle nh_;
  mutable interactive_markers::InteractiveMarkerServer server_;
};

}  // namespace drake_iiwa_sim
