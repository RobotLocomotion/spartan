#include <drake_robot_control/joint_space_trajectory_plan.h>

namespace drake {
namespace robot_plan_runner {

// Current robot state x = [q,v]
// Current time t
void JointSpaceTrajectoryPlan::Step(
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

  DRAKE_ASSERT(t >= 0);
  *q_commanded = traj_.value(t);
  *v_commanded = traj_d_.value(t);
  *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());

  if (t > this->duration()) {

    if (plan_status_.compare_exchange_strong(running_status,
                                             PlanStatus::FINISHED_NORMALLY)) {
      ROS_INFO("plan finished normally");

      // notify the condition variable
      this->SetPlanFinished();
    }
  }
}

std::unique_ptr<JointSpaceTrajectoryPlan>
JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(std::shared_ptr<const RigidBodyTreed> tree,
                            const Eigen::Ref<const Eigen::VectorXd> &q) {
  // creates a zero-order hold around current robot configuration q.
  const int nq = tree->get_num_positions();
  DRAKE_ASSERT(nq == q.rows());
  std::vector<double> times{0, 1};
  std::vector<Eigen::MatrixXd> knots(2, Eigen::MatrixXd::Zero(nq, 1));
  for (int i = 0; i < nq; i++) {
    knots[0](i, 0) = q[i];
    knots[1](i, 0) = q[i];
  }
  auto ptr = std::make_unique<JointSpaceTrajectoryPlan>(
      tree, PPType::ZeroOrderHold(times, knots));
  return std::move(ptr);
}


} // namespace robot_plan_runner
} // namespace drake