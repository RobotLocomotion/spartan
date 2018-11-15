#include "drake_iiwa_sim/schunk_wsg_ros.h"

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake_iiwa_sim {

SchunkWsgActionServer::SchunkWsgActionServer(double initial_position,
                                             double initial_force)
    : latest_commanded_position_(initial_position),
      latest_commanded_force_(initial_force),
      measured_position_input_port_(
        this->DeclareVectorInputPort(
          "measured_position", BasicVector<double>(1)).get_index()),
      measured_force_input_port_(
        this->DeclareVectorInputPort(
          "measured_force", BasicVector<double>(1)).get_index()),
      position_output_port_(
          this->DeclareVectorOutputPort(
                  "position", BasicVector<double>(1),
                  &SchunkWsgActionServer::CalcPositionOutput)
              .get_index()),
      force_limit_output_port_(
          this->DeclareVectorOutputPort(
                  "force_limit", BasicVector<double>(1),
                  &SchunkWsgActionServer::CalcForceLimitOutput)
              .get_index()),
      as_(nh_, "WSGCommandTester", false) {
  this->DeclarePeriodicPublish(0.01);
  as_.start();

}

void SchunkWsgActionServer::DoPublish(
    const Context<double>& context,
    const std::vector<const systems::PublishEvent<double>*>& events) const {
  ros::spinOnce();

  // Inputs
  if (as_.isNewGoalAvailable()){
    auto goal = as_.acceptNewGoal();
    latest_commanded_position_ = goal->width;
    //goal->speed
    latest_commanded_force_ = goal->force;
   }

  // Outputs
  if (as_.isActive()){
    //
  }
}

void SchunkWsgActionServer::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  if (std::isnan(latest_commanded_position_)) {
    latest_commanded_position_ = 0;
  }
  output->SetAtIndex(0, latest_commanded_position_);
}

void SchunkWsgActionServer::CalcForceLimitOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  if (std::isnan(latest_commanded_force_)) {
    latest_commanded_force_ = 0;
  }

  output->SetAtIndex(0, latest_commanded_force_);
}

}  // namespace drake_iiwa_sim