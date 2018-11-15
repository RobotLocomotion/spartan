#include "drake_iiwa_sim/schunk_wsg_ros_actionserver.h"

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"

namespace drake_iiwa_sim {

using drake::systems::BasicVector;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::Context;
using drake::systems::State;

SchunkWsgActionServer::SchunkWsgActionServer(double initial_position,
                                             double initial_force_limit)
    : initial_position_(initial_position),
      initial_force_limit_(initial_force_limit),
      min_commanded_position_(0.),
      max_commanded_position_(0.1), // Magic #...
      measured_state_input_port_(
        this->DeclareVectorInputPort(
          "measured_state", BasicVector<double>(1)).get_index()),
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
  // State[0]: latest commanded position
  // State[1]: latest commanded force limit
  this->DeclareDiscreteState(2);
  this->DeclarePeriodicDiscreteUpdate(0.01);

  as_.start();

}

void SchunkWsgActionServer::SetDefaultState(
    const Context<double>&, State<double>* state) const {
  state->get_mutable_discrete_state().get_mutable_vector().set_value(
    Eigen::Vector2d(initial_position_, initial_force_limit_));
}

void SchunkWsgActionServer::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  BasicVector<double>& state_value = discrete_state->get_mutable_vector(0);

  
  const auto measured_state = this->EvalVectorInput(
    context, measured_state_input_port_)->get_value();
  const double measured_force = this->EvalVectorInput(
    context, measured_force_input_port_)->GetAtIndex(0);

  ros::spinOnce();

  // Inputs
  if (as_.isNewGoalAvailable()){
    auto goal = as_.acceptNewGoal();
    state_value[0] = goal->width;
    state_value[1] = goal->force;
  }

  wsg_50_common::Status status;
  status.width = measured_state[0];
  status.current_speed = measured_state[1];
  status.current_force = measured_force;
  status.grasping_force = state_value[1];
  
  if (as_.isPreemptRequested()){
    as_.setPreempted(status);
  }

  if (abs(measured_state[0] - state_value[0]) < 0.001 ){
    as_.setSucceeded(status);
  }

  if (as_.isActive()){
    as_.publishFeedback(status);
  }
}

void SchunkWsgActionServer::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  if (std::isnan(latest_commanded_position_)) {
    latest_commanded_position_ = 0;
  }
  latest_commanded_position_ = fmin(fmax(latest_commanded_position_,
                                         min_commanded_position_),
                                    max_commanded_position_);
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