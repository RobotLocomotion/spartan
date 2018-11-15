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

class SchunkWsgActionServer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgActionServer)

  /// Contains a SimpleActionServer server spoofing the
  /// real WSG50 driver used in Spartan.

  /// @param initial_position the commanded position to output if no command
  /// message has been received yet.
  ///
  /// @param initial_force the commanded force limit to output if no command
  /// message has been received yet.
  SchunkWsgActionServer(double initial_position = 0.02,
                           double initial_force = 40);

  const systems::InputPort<double>& get_measured_position_input_port() const {
    return this->get_input_port(measured_position_input_port_);
  }

  const systems::InputPort<double>& get_measured_force_input_port() const {
    return this->get_input_port(measured_force_input_port_);
  }

  const systems::OutputPort<double>& get_position_output_port() const {
    return this->get_output_port(position_output_port_);
  }

  const systems::OutputPort<double>& get_force_limit_output_port() const {
    return this->get_output_port(force_limit_output_port_);
  }

 private:
  void CalcPositionOutput(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;

  void CalcForceLimitOutput(const systems::Context<double>& context,
                            systems::BasicVector<double>* output) const;

 private:
  double latest_commanded_position_;
  double latest_commanded_force_;
  const systems::InputPortIndex measured_position_input_port_{};
  const systems::InputPortIndex measured_force_limit_input_port_{};
  const systems::OutputPortIndex position_output_port_{};
  const systems::OutputPortIndex force_limit_output_port_{};
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<wsg_50_common::CommandAction> as_;

};

}  // namespace drake_iiwa_sim
