#include "drake_iiwa_sim/schunk_wsg_ros_actionserver.h"

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"

namespace drake_iiwa_sim {

using drake::systems::BasicVector;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::Context;
using drake::systems::State;

SchunkWsgActionServer::SchunkWsgActionServer(std::string name,
                                             std::string status_channel,
                                             double initial_position,
                                             double initial_force_limit)
    : initial_position_(initial_position),
      initial_force_limit_(initial_force_limit),
      min_commanded_position_(0.),
      max_commanded_position_(0.1), // Magic #...
      measured_state_input_port_(
        this->DeclareVectorInputPort(
          "measured_state", BasicVector<double>(2)).get_index()),
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
      as_(nh_, name, false) {
  if (status_channel.size() > 0){
    pb_ = nh_.advertise<wsg_50_common::Status>(status_channel, 1);
    do_publish_ = true;
  } else {
    do_publish_ = false;
  }

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

void SchunkWsgActionServer::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const DiscreteUpdateEvent<double>*>&,
    DiscreteValues<double>* discrete_state) const {
  BasicVector<double>& state_value = discrete_state->get_mutable_vector(0);

  ros::spinOnce();
  
  const auto measured_state = this->EvalVectorInput(
    context, measured_state_input_port_)->get_value();
  const double measured_force = this->EvalVectorInput(
    context, measured_force_input_port_)->GetAtIndex(0);

  // Inputs
  if (as_.isNewGoalAvailable()){
    auto goal = as_.acceptNewGoal();

    state_value[0] = fmin(fmax(goal->command.width,
                             min_commanded_position_),
                          max_commanded_position_);
    state_value[1] = goal->command.force;

    for (int i = 0; i < 2; i++){
      if (std::isnan(state_value[0])){
        state_value[0] = 0.;
      }
    }
  }

  wsg_50_common::Status status;
  status.stamp = ros::Time::now();
  status.width = measured_state[0];
  status.current_speed = measured_state[1];
  status.current_force = measured_force;
  status.grasping_force = state_value[1];
  
  if (do_publish_){
    pb_.publish(status);
  }

  if (as_.isActive()){
    wsg_50_common::CommandResult result;    
    result.status = status;

    if (as_.isPreemptRequested()){
      as_.setPreempted(result);
    }

    if (abs(measured_state[0] - state_value[0]) < 0.001 ){
      as_.setSucceeded(result);
    }

    wsg_50_common::CommandFeedback feedback;
    feedback.status = status;
    as_.publishFeedback(feedback);
  }
}

void SchunkWsgActionServer::CalcPositionOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& state_value = context.get_discrete_state_vector();
  output->SetAtIndex(0, state_value[0]);
}

void SchunkWsgActionServer::CalcForceLimitOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& state_value = context.get_discrete_state_vector();
  output->SetAtIndex(0, state_value[1]);
}

}  // namespace drake_iiwa_sim