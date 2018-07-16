#pragma once

#include <drake/multibody/rigid_body_tree.h>

namespace drake {
namespace robot_plan_runner {

// Abstract class
// Child classes of PlanBase should have a concrete Step() method which
// generates the commanded state/torque.

// TODO: add support for all Plans to return PlanStatus;
enum PlanStatus {
  NOT_STARTED,
  RUNNING,
  FINISHED_NORMALLY,
  STOPPED_BY_EXTERNAL_TRIGGER,
  STOPPED_BY_SAFETY_CHECK
};

class PlanBase {
 public:
  explicit PlanBase(std::shared_ptr<const RigidBodyTreed> tree)
      : tree_(std::move(tree)) {
    num_positions = tree_->get_num_positions();
    num_velocities = tree_->get_num_velocities();
    plan_status_ = NOT_STARTED;
  }

  // x:=[q,v] robot state
  // t: plan time (relative to plan start time)
  virtual void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &tau_external,
                    double t,
                    Eigen::VectorXd *const q_commanded,
                    Eigen::VectorXd *const v_commanded,
                    Eigen::VectorXd *const tau_commanded) = 0;
  int get_num_positions() const { return num_positions; }
  int get_num_velocities() const { return num_velocities; }
  PlanStatus get_plan_status() const { return plan_status_; }
 protected:
  std::shared_ptr<const RigidBodyTreed> tree_;

 private:
  int num_positions;
  int num_velocities;
  PlanStatus plan_status_;
};

} // namespace robot_plan_runner
} // namespace drake
