#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <drake/multibody/rigid_body_tree.h>

#include "robot_msgs/PlanStatus.h"



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
  STOPPED_BY_SAFETY_CHECK,
  STOPPPED_BY_FORCE_THRESHOLD,
};

class PlanBase {
 public:
  explicit PlanBase(std::shared_ptr<const RigidBodyTreed> tree)
      : tree_(std::move(tree)), is_finished_(false) {
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

  // return (a copy) of the current plan status
  PlanStatus get_plan_status() const { return plan_status_.load();}
  void Stop(){};
  PlanStatus WaitForPlanToFinish();

  // sets the plan to be finished, notifies any waiting threads
  void SetPlanFinished();

  void GetPlanStatusMsg(robot_msgs::PlanStatus& plan_status_msg);

  
  // for multi-thread synchronization
  std::atomic<PlanStatus> plan_status_;
  std::condition_variable cv_;
  std::mutex mutex_;
  bool is_finished_;
  int plan_number_;

 protected:
  std::shared_ptr<const RigidBodyTreed> tree_;
  
 private:
  int num_positions;
  int num_velocities;
  
  

  ;
};

} // namespace robot_plan_runner
} // namespace drake
