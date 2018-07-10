#pragma once

#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <robot_plan_runner/plan_base.h>
#include <robot_plan_runner/task_space_trajectory_plan.h>
#include <robot_plan_runner/joint_space_trajectory_plan.h>

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

const char *const kLcmStatusChannel = "IIWA_STATUS";
const char *const kLcmCommandChannel = "IIWA_COMMAND";
const int kNumJoints = 7;

class RobotPlanRunner {
public:
  RobotPlanRunner();
  ~RobotPlanRunner() { Stop(); }

  void Start();
  void Stop();

  // The following methods locks robot_status_mutex and returns current robot
  // state/position/velocity.
  Eigen::VectorXd get_current_robot_state();
  Eigen::VectorXd get_current_robot_position();
  Eigen::VectorXd get_current_robot_velocity();

  Eigen::Isometry3d get_ee_pose_in_world_frame() {
    return get_body_pose_in_world_frame(*tree_->FindBody("iiwa_link_ee"));
  }

  // This method updates new_plan_ from a nullptr to point to a new PlanBase
  // object.
  // At the beginning of every command publisher loop, PlanRunner checks whether
  // new_plan_ is a nullptr.
  // If new_plan_ is not a nullptr, it is moved to plan_(which will be executed
  // immediately), and becomes a nullptr again.
  void QueueNewPlan(std::unique_ptr<PlanBase> new_plan) {
    std::lock_guard<std::mutex> lock(robot_plan_mutex_);
    new_plan_ = std::move(new_plan);
  }

  // worker method of lcm status receiver thread.
  void ReceiveRobotStatus();

  // worker method of lcm command publihser thread.
  void PublishCommand();

  // This method is used by the condition_variable to prevent spurious wakeups.
  bool has_received_new_status() { return has_received_new_status_; }

  void MoveToJointPosition(const Eigen::Ref<const Eigen::VectorXd> q_final,
                           double duration = 4);

  void MoveRelativeToCurrentEeCartesianPosition(
      const Eigen::Ref<const Eigen::Vector3d> delta_x_ee, double duration = 4);

private:
  void HandleStatus(const lcm::ReceiveBuffer *, const std::string &,
                    const lcmt_iiwa_status *status);

  // This is a leftover method from the old iiwa_plan_runner.
  // We need a way to stop current command but it has not been implemented.
  void HandleStop(const lcm::ReceiveBuffer *, const std::string &,
                  const robotlocomotion::robot_plan_t *) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  Eigen::Isometry3d get_body_pose_in_world_frame(const RigidBody<double> &body);

  std::shared_ptr<RigidBodyTreed> tree_;
  int plan_number_{};

  // threading
  std::mutex robot_status_mutex_;
  std::mutex robot_plan_mutex_;
  std::thread publish_thread_;
  std::thread subscriber_thread_;
  std::condition_variable cv_;

  // subscriber_thread
  lcm::LCM receiver_lcm_;
  bool waiting_for_first_message_;

  // publisher thread
  lcm::LCM publihser_lcm_;
  std::unique_ptr<PlanBase> plan_;

  // shared
  lcmt_iiwa_status iiwa_status_;
  bool has_received_new_status_;
  Eigen::VectorXd current_robot_state_;
  int64_t cur_time_us_ = -1;
  std::unique_ptr<PlanBase> new_plan_;

};

} // namespace examples
} // namespace drake