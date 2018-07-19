#pragma once
#include <drake_robot_control/trajectory_plan_base.h>

// ROS
#include <actionlib/server/simple_action_server.h>
#include "robot_msgs/JointTrajectoryAction.h"

namespace drake {
namespace robot_plan_runner {

class JointSpaceTrajectoryPlan : public TrajectoryPlanBase {
public:
  JointSpaceTrajectoryPlan(std::shared_ptr<const RigidBodyTreed> tree,
                           const PPType &q_traj)
      : TrajectoryPlanBase(std::move(tree), q_traj) {
    DRAKE_ASSERT(q_traj.rows() == get_num_positions());
  }

  // Current robot state x = [q,v]
  // Current time t
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external,
            double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) override {

    PlanStatus not_started_status = PlanStatus::NOT_STARTED;
    PlanStatus running_status = PlanStatus::RUNNING;

    plan_status_.compare_exchange_strong(not_started_status, PlanStatus::RUNNING);

    DRAKE_ASSERT(t >= 0);
    *q_commanded = traj_.value(t);
    *v_commanded = traj_d_.value(t);
    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());

    if (t > traj_.end_time()){

      if (plan_status_.compare_exchange_strong(running_status, PlanStatus::FINISHED_NORMALLY)){
        ROS_INFO("plan finished normally");

        // notify the condition variable
        this->SetPlanFinished();

        // // update the status via the ROS action if the
        // // pointer is not null
        // if (joint_trajectory_action_){
        //   ROS_INFO("setting ROS action to succeeded state");
        //   robot_msgs::JointTrajectoryResult result;
        //   result.status.status = result.status.FINISHED_NORMALLY;
        //   joint_trajectory_action_->setSucceeded();
        // }
      }
    }
  }

  static std::unique_ptr<JointSpaceTrajectoryPlan>
  MakeHoldCurrentPositionPlan(std::shared_ptr<const RigidBodyTreed> tree,
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

  void SetActionServer(std::shared_ptr<actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>> action_pointer){
    joint_trajectory_action_ = action_pointer;
  }

  std::shared_ptr<actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>> joint_trajectory_action_;
};

} // namespace robot_plan_runner
} // namespace drake