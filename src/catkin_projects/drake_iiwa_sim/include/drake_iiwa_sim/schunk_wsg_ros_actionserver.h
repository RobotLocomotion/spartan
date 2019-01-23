#pragma once

/// @file This file contains classes dealing with sending/receiving
/// ROS messages related to the Schunk WSG gripper that spoofs a
/// real gripper interface (as used in Spartan) using outputs
/// from a simulated gripper.

#include "drake/common/drake_deprecated.h"
#include "drake/systems/framework/leaf_system.h"

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "wsg_50_common/CommandAction.h"


namespace drake_iiwa_sim {

class SchunkWsgActionServer : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgActionServer)

  /// Contains a SimpleActionServer server spoofing the
  /// real WSG50 driver used in Spartan.

  /// @param initial_position the commanded position to output if no command
  /// message has been received yet.
  ///
  /// @param initial_force the commanded force limit to output if no command
  /// message has been received yet.
  SchunkWsgActionServer(std::string name,
                        std::string status_channel = "",
                        double initial_position = 0.02,
                        double initial_force_limit = 40);

  const drake::systems::InputPort<double>& get_measured_state_input_port() const {
    return this->get_input_port(measured_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_measured_force_input_port() const {
    return this->get_input_port(measured_force_input_port_);
  }

  const drake::systems::OutputPort<double>& get_position_output_port() const {
    return this->get_output_port(position_output_port_);
  }

  const drake::systems::OutputPort<double>& get_force_limit_output_port() const {
    return this->get_output_port(force_limit_output_port_);
  }

  drake::optional<bool> DoHasDirectFeedthrough(int, int) const final { return false; }

 protected:
  void SetDefaultState(const drake::systems::Context<double>& context,
                       drake::systems::State<double>* state) const override;

  void DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<double>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<double>* discrete_state) const override;

 private:
  void CalcPositionOutput(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* output) const;

  void CalcForceLimitOutput(const drake::systems::Context<double>& context,
                            drake::systems::BasicVector<double>* output) const;

 private:
  const double initial_position_;
  const double initial_force_limit_;
  const double max_commanded_position_;
  const double min_commanded_position_;
  const drake::systems::InputPortIndex measured_state_input_port_{};
  const drake::systems::InputPortIndex measured_force_input_port_{};
  const drake::systems::OutputPortIndex position_output_port_{};
  const drake::systems::OutputPortIndex force_limit_output_port_{};
  mutable ros::NodeHandle nh_;
  mutable actionlib::SimpleActionServer<wsg_50_common::CommandAction> as_;
  mutable ros::Publisher pb_;
  bool do_publish_;
};

}  // namespace drake_iiwa_sim
