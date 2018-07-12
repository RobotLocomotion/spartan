#pragma once

#include <drake/multibody/rigid_body_tree.h>

namespace drake {
namespace robot_plan_runner {

// Abstract class
// Child classes of PlanBase should have a concrete Step() method which
// generates the commanded state/torque.
class PlanBase {
 public:
  explicit PlanBase(std::shared_ptr<const RigidBodyTreed> tree)
      : tree_(std::move(tree)) {
    num_positions = tree_->get_num_positions();
    num_velocities = tree_->get_num_velocities();
  }

  // x:=[q,v] robot state
  // t: plan time (relative to plan start time)
  virtual void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &tau_external,
                    double t,
                    Eigen::VectorXd *const q_commanded,
                    Eigen::VectorXd *const v_commanded,
                    Eigen::VectorXd *const tau_commanded) const = 0;
  int get_num_positions() const { return num_positions; }
  int get_num_velocities() const { return num_velocities; }

 protected:
  std::shared_ptr<const RigidBodyTreed> tree_;

 private:
  int num_positions;
  int num_velocities;
};

} // namespace robot_plan_runner
} // namespace drake