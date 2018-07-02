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
// Every Plan subclass should have a concrete Run method which generates the
// commanded state/torque.
class Plan {
public:
  Plan() : nq_(-1) {}
  Plan(std::shared_ptr<const RigidBodyTreed> tree, PlanType p_type)
      : tree_(tree), p_type_(p_type), nq_(tree->get_num_positions()) {}

  PlanType get_plan_type() { return p_type_; }
  int get_num_dofs() { return nq_; }

  // every subclass should override the Step method with a concrete
  // implementation.
  virtual void Step(const Eigen::Ref<const Eigen::VectorXd> &x, double t,
                    Eigen::VectorXd *const q_commanded,
                    Eigen::VectorXd *const v_commanded) const {
    throw std::runtime_error("Step() in the base class Plan has been called.");
  }

  double duration() {
    if (q_traj_.get_number_of_segments() > 0) {
      return q_traj_.end_time() - q_traj_.start_time();
    } else {
      return 0.;
    }
  }

protected:
  PPType q_traj_;
  PPType v_traj_;
  std::shared_ptr<const RigidBodyTreed> tree_;

private:
  const int nq_{-1};
  PlanType p_type_;
};

class JointSpaceTrajectoryPlan : public Plan {
public:
  JointSpaceTrajectoryPlan(std::shared_ptr<const RigidBodyTreed> tree,
                           const PPType &q_traj)
      : Plan(tree, JointSpaceTrajectoryPlanType) {
    DRAKE_ASSERT(q_traj.cols() == 1);
    DRAKE_ASSERT(q_traj.rows() == this->get_num_dofs());
    q_traj_ = q_traj;
    v_traj_ = q_traj_.derivative(1);
  }

  // Current robot state x = [q,v]
  // Current time t
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded) const override {
    DRAKE_ASSERT(t >= 0);
    *q_commanded = q_traj_.value(t);
    *v_commanded = v_traj_.value(t);
  }

  static std::unique_ptr<JointSpaceTrajectoryPlan>
  MakeBlankPlan(std::shared_ptr<const RigidBodyTreed> tree,
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

} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake