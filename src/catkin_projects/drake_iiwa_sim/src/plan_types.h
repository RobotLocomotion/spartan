#pragma once

#include <stdexcept>

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/rigid_body_tree.h>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

enum PlanType { JointSpaceTrajectoryPlanType, TaskSpaceTrajectoryPlanType };
typedef trajectories::PiecewisePolynomial<double> PPType;

// abstract class
// Every Plan subclass should have a concrete Step() method which generates the
// commanded state/torque.
class Plan {
public:
  Plan(std::shared_ptr<const RigidBodyTreed> tree, PlanType p_type):
      tree_(tree), p_type_(p_type) {};
  virtual void Step(const Eigen::Ref<const Eigen::VectorXd> &x, double t,
                    Eigen::VectorXd *const q_commanded,
                    Eigen::VectorXd *const v_commanded) const = 0;
  int get_num_positions() { return tree_->get_num_positions(); };
  PlanType get_plan_type() { return p_type_; }

protected:
  PlanType p_type_;
  std::shared_ptr<const RigidBodyTreed> tree_;
};

class TrajectoryPlan : Plan {
public:
  TrajectoryPlan(std::shared_ptr<const RigidBodyTreed> tree,
                  PlanType p_type, const PPType &q_traj)
      : Plan(tree, p_type), traj_(q_traj) {
    DRAKE_ASSERT(q_traj.cols() == 1);
    DRAKE_ASSERT(q_traj.rows() == get_num_positions());
    traj_d_ = traj_.derivative(1);
  }

  double duration() {
    if (traj_.get_number_of_segments() > 0) {
      return traj_.end_time() - traj_.start_time();
    } else {
      return 0.;
    }
  }

protected:
  PPType traj_;
  PPType traj_d_; // 1st order derivative of traj_
};

class JointSpaceTrajectoryPlan : public TrajectoryPlan {
public:
  JointSpaceTrajectoryPlan(std::shared_ptr<const RigidBodyTreed> tree,
                           const PPType &q_traj)
      : TrajectoryPlan(tree, JointSpaceTrajectoryPlanType, q_traj) {}

  // Current robot state x = [q,v]
  // Current time t
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded) const override {
    DRAKE_ASSERT(t >= 0);
    *q_commanded = traj_.value(t);
    *v_commanded = traj_d_.value(t);
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

class TaskSpaceTrajectoryPlan : public TrajectoryPlan {
  TaskSpaceTrajectoryPlan(std::shared_ptr<const RigidBodyTreed> tree,
                          const PPType &x_ee_traj)
      : TrajectoryPlan(tree, TaskSpaceTrajectoryPlanType, x_ee_traj) {}
};

} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake