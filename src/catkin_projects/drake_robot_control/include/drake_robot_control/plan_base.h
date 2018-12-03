#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "robot_msgs/PlanStatus.h"
#include <drake/multibody/rigid_body_tree.h>

#include "drake_robot_control/force_guard.h"

namespace drake {
namespace robot_plan_runner {

typedef spartan::drake_robot_control::ForceGuardContainer ForceGuardContainer;
typedef spartan::drake_robot_control::ForceGuard ForceGuard;

// Abstract class
// Child classes of PlanBase should have a concrete Step() method which
// generates the commanded state/torque.

// TODO: add support for all Plans to return PlanStatus;
enum PlanStatus {
  NOT_STARTED,
  RUNNING,
  FINISHED_NORMALLY,
  STOPPED_BY_EXTERNAL_TRIGGER,
  STOPPED_BY_SAFETY_CHECK,
  STOPPED_BY_FORCE_GUARD,
};

class PlanBase {
public:
  explicit PlanBase(std::shared_ptr<const RigidBodyTreed> tree)
      : tree_(std::move(tree)), cache_(tree_->CreateKinematicsCache()),
        cache_measured_state_(tree_->CreateKinematicsCache()),
        is_finished_(false) {
    num_positions = tree_->get_num_positions();
    num_velocities = tree_->get_num_velocities();
    plan_status_ = NOT_STARTED;
  }

  // x:=[q,v] robot state
  // t: plan time (relative to plan start time)
  virtual void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &tau_external,
                    double t, Eigen::VectorXd *const q_commanded,
                    Eigen::VectorXd *const v_commanded,
                    Eigen::VectorXd *const tau_commanded) = 0;

  int get_num_positions() const { return num_positions; }
  int get_num_velocities() const { return num_velocities; }

  inline void set_plan_status(PlanStatus plan_status) {
    plan_status_ = plan_status;
  }

  // return (a copy) of the current plan status
  PlanStatus get_plan_status() const { return plan_status_.load(); }


  inline void set_max_dq_per_step(const double max_dq_per_step) {
    max_dq_per_step_ = max_dq_per_step;
  }

  double get_max_dq_per_step() const { return max_dq_per_step_; }

  // return true if plan is stopped
  // Note: finished does not mean stopped
  inline bool is_stopped() {
    bool plan_halted = false;
    PlanStatus plan_status = this->get_plan_status();

    if ((plan_status_ == PlanStatus::STOPPED_BY_EXTERNAL_TRIGGER) ||
        (plan_status == PlanStatus::STOPPED_BY_SAFETY_CHECK) ||
        (plan_status == PlanStatus::STOPPED_BY_FORCE_GUARD)) {
      plan_halted = true;
    }

    return plan_halted;
  }

  void Stop(){}; // currently does nothing
  PlanStatus WaitForPlanToFinish();

  // sets the plan to be finished, notifies any waiting threads
  void SetPlanFinished();

  void GetPlanStatusMsg(robot_msgs::PlanStatus &plan_status_msg);

  // store the previously commanded q, tau
  inline void
  SetCurrentCommand(const Eigen::Ref<const Eigen::VectorXd> &q_commanded,
                    const Eigen::Ref<const Eigen::VectorXd> &tau_commanded) {
    q_commanded_prev_ = q_commanded;
    tau_commanded_prev_ = tau_commanded;
  }

  inline void
  set_guard_container(std::shared_ptr<ForceGuardContainer> guard_container) {
    guard_container_ = guard_container;
  }

  // for multi-thread synchronization
  std::atomic<PlanStatus> plan_status_;
  std::condition_variable cv_;
  std::mutex mutex_;
  bool is_finished_;
  int plan_number_;

protected:
  std::shared_ptr<const RigidBodyTreed> tree_;
  KinematicsCache<double> cache_;
  KinematicsCache<double> cache_measured_state_;

  // records the last commands sent by this plan
  // or the last command sent by a previous plan if we have just swapped this
  // plan in
  Eigen::VectorXd q_commanded_prev_;
  Eigen::VectorXd tau_commanded_prev_;
  std::shared_ptr<ForceGuardContainer> guard_container_;

private:
  int num_positions;
  int num_velocities;
  double max_dq_per_step_;

  ;
};

} // namespace robot_plan_runner
} // namespace drake
