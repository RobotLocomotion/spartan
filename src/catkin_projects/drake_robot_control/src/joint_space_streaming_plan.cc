#include <drake_robot_control/joint_space_streaming_plan.h>

namespace drake {
namespace robot_plan_runner {

// Current robot state x = [q,v]
// Current time t
void JointSpaceStreamingPlan::Step(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
    Eigen::VectorXd *const q_commanded, Eigen::VectorXd *const v_commanded,
    Eigen::VectorXd *const tau_commanded) {

  PlanStatus not_started_status = PlanStatus::NOT_STARTED;
  PlanStatus running_status = PlanStatus::RUNNING;

  plan_status_.compare_exchange_strong(not_started_status, PlanStatus::RUNNING);

  // check if the plan has been stopped
  // if so just echo the last command
  if (this->is_stopped()) {
    *q_commanded = q_commanded_prev_;
    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
    return;
  }

  // do kinematics so we can check the force guards
  Eigen::VectorXd q = x.head(this->get_num_positions());
  Eigen::VectorXd v = x.tail(this->get_num_positions());
  cache_measured_state_.initialize(q, v);
  tree_->doKinematics(cache_measured_state_);

  // Check the external force guards
  if (guard_container_) {
    std::pair<bool, std::pair<double, std::shared_ptr<ForceGuard>>> result =
                                          guard_container_->EvaluateGuards(cache_measured_state_, q,
                                                                           tau_external);

    bool guard_triggered = result.first;

    if (guard_triggered) {
      std::cout << "Force Guard Triggered, stopping plan" << std::endl;
      std::cout << "ForceGuardType: " << result.second.second->get_type()
                << std::endl;
      plan_status_ = PlanStatus::STOPPED_BY_FORCE_GUARD;
      this->SetPlanFinished();

      *q_commanded = q_commanded_prev_;
      *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
      return;
    }
  }

  // 
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (q_commanded_.rows() == q_commanded->rows() &&
      v_commanded_.rows() == v_commanded->rows() &&
      tau_commanded_.rows() == tau_commanded->rows()){
    *q_commanded = q_commanded_;
    *v_commanded = v_commanded_;
    *tau_commanded = tau_commanded_;
  } else {
   *q_commanded = q_commanded_prev_;
   *tau_commanded =  tau_commanded_prev_;
  }
}

void JointSpaceStreamingPlan::HandleSetpoint(const sensor_msgs::JointState::ConstPtr& msg) {
  std::map<std::string, int> name_to_idx =
    tree_->computePositionNameToIndexMap();

  DRAKE_ASSERT(msg->position.size() == msg->name.size());
  DRAKE_ASSERT(msg->velocity.size() == msg->name.size());
  DRAKE_ASSERT(msg->effort.size() == msg->name.size());
  DRAKE_ASSERT(msg->name.size() == this->get_num_positions());

  Eigen::VectorXd q = Eigen::VectorXd::Zero(this->get_num_positions());
  Eigen::VectorXd v = Eigen::VectorXd::Zero(this->get_num_velocities());
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(this->get_num_positions());

  for (int i = 0; i < msg->name.size(); i++) {
    if (name_to_idx.count(msg->name[i]) == 0) {
      continue;
    }
    int ind = name_to_idx[msg->name[i]];
    DRAKE_ASSERT(ind < this->get_num_positions());
    DRAKE_ASSERT(ind < this->get_num_velocities());
    q[ind] = msg->position[i];
    v[ind] = msg->velocity[i];
    tau[ind] = msg->effort[i];
  }
  SetGoal(q, v, tau);
}

} // namespace robot_plan_runner
} // namespace drake