#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <drake_robot_control/joint_space_trajectory_plan.h>
#include <drake_robot_control/plan_base.h>
#include <drake_robot_control/task_space_trajectory_plan.h>

#include <lcm/lcm-cpp.hpp>
#include <robotlocomotion/robot_plan_t.hpp>

#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/lcmt_iiwa_command.hpp>
#include <drake/lcmt_iiwa_status.hpp>
#include <drake/multibody/joints/floating_base_types.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_tree.h>

namespace drake {
namespace robot_plan_runner {

class RobotPlanRunner {
public:
  // The constructor should not be called directly.
  // GetInstance should be called to create an instance of RobotPlanRunner from a config file.
  static std::unique_ptr<RobotPlanRunner>
  GetInstance(const std::string &config_file_name);

  RobotPlanRunner(const std::string &lcm_status_channel,
                  const std::string &lcm_command_channel,
                  const std::string &lcm_plan_channel,
                  const std::string &lcm_stop_channel,
                  const std::string &robot_ee_body_name, int num_joints,
                  double joint_speed_limit_deg_per_sec, double control_period,
                  std::unique_ptr<const RigidBodyTreed> tree);
  ~RobotPlanRunner();

  void Start();

  // The following methods locks robot_status_mutex and returns current robot
  // state/position/velocity.
  Eigen::VectorXd get_current_robot_state();
  Eigen::VectorXd get_current_robot_position();
  Eigen::VectorXd get_current_robot_velocity();

  // This method updates new_plan_ from a nullptr to point to a new PlanBase
  // object.
  // At the beginning of every command publisher loop, PlanRunner checks whether
  // new_plan_ is a nullptr.
  // If new_plan_ is not a nullptr, it is moved to plan_local (which will be executed
  // immediately), and becomes a nullptr again.
  void QueueNewPlan(std::unique_ptr<PlanBase> new_plan) {
    std::lock_guard<std::mutex> lock(robot_plan_mutex_);
    new_plan_ = std::move(new_plan);
  }

  std::shared_ptr<const RigidBodyTreed> get_rigid_body_tree() { return tree_; }
  double get_control_period() { return kControlPeriod_; }
  std::string get_ee_body_name() { return kRobotEeBodyName_; }

  Eigen::Isometry3d get_ee_pose_in_world_frame() {
    return get_body_pose_in_world_frame(*tree_->FindBody(kRobotEeBodyName_));
  }

  void MoveToJointPosition(const Eigen::Ref<const Eigen::VectorXd> q_final,
                           double duration = 4);

  void MoveRelativeToCurrentEeCartesianPosition(
      const Eigen::Ref<const Eigen::Vector3d> delta_x_ee, double duration = 4);

private:
  // worker method of lcm status receiver thread.
  void ReceiveRobotStatus();

  // worker method of lcm command publisher thread.
  void PublishCommand();

  // worker method of lcm construct new plan thread.
  // This thread should subscribe to a channel with LCM type robot_plant_t and
  // generate JointSpaceTrajectoryPlan.
  void ConstructNewPlanFromLcm();

  // This method is used by the condition_variable to prevent spurious wakeups.
  bool has_received_new_status() { return has_received_new_status_; }

  void HandleStatus(const lcm::ReceiveBuffer *, const std::string &,
                    const lcmt_iiwa_status *status);

  void
  HandleJointSpaceTrajectoryPlan(const lcm::ReceiveBuffer *,
                                 const std::string &,
                                 const robotlocomotion::robot_plan_t *tape);

  void HandleStop(const lcm::ReceiveBuffer *, const std::string &,
                  const robotlocomotion::robot_plan_t *) {
    std::lock_guard<std::mutex> lock(robot_plan_mutex_);
    is_plan_terminated_externally_ = true;
    new_plan_.reset();
  }

  Eigen::Isometry3d get_body_pose_in_world_frame(const RigidBody<double> &body);

  // constants to be loaded from yaml file.
  const std::string kLcmStatusChannel_;
  const std::string kLcmCommandChannel_;
  const std::string kLcmPlanChannel_;
  const std::string kLcmStopChannel_;
  const std::string kRobotEeBodyName_;
  const int kNumJoints_;
  const double kJointSpeedLimitDegPerSec_;
  const double kControlPeriod_;

  std::shared_ptr<const RigidBodyTreed> tree_;
  int plan_number_{};

  // threading
  std::mutex robot_status_mutex_;
  std::mutex robot_plan_mutex_;
  std::thread publish_thread_;
  std::thread subscriber_thread_;
  std::thread plan_constructor_thread_;
  std::condition_variable cv_;

  std::atomic<bool> is_waiting_for_first_robot_status_message_;
  std::atomic<bool> has_received_new_status_;
  std::atomic<bool> is_plan_terminated_externally_;
  lcmt_iiwa_status iiwa_status_;
  Eigen::VectorXd current_robot_state_;
  std::unique_ptr<PlanBase> new_plan_;
};

} // namespace examples
} // namespace drake