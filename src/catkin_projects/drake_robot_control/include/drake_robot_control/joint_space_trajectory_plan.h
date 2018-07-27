#pragma once
#include <drake_robot_control/trajectory_plan_base.h>

// ROS
#include "robot_msgs/JointTrajectoryAction.h"
#include <actionlib/server/simple_action_server.h>

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
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) override;

  static std::unique_ptr<JointSpaceTrajectoryPlan>
  MakeHoldCurrentPositionPlan(std::shared_ptr<const RigidBodyTreed> tree,
                              const Eigen::Ref<const Eigen::VectorXd> &q);

  inline void SetActionServer(
      std::shared_ptr<
          actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>>
          action_pointer) {
    joint_trajectory_action_ = action_pointer;
  }

 private:
  std::shared_ptr<
      actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>>
      joint_trajectory_action_;
};

} // namespace robot_plan_runner
} // namespace drake