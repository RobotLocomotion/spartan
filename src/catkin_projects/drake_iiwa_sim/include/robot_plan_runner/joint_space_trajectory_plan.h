#pragma once
#include <robot_plan_runner/trajectory_plan_base.h>

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
            Eigen::VectorXd *const tau_commanded) const override {
    DRAKE_ASSERT(t >= 0);
    *q_commanded = traj_.value(t);
    *v_commanded = traj_d_.value(t);
    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
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
};

} // namespace robot_plan_runner
} // namespace drake