#pragma once

#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "plan_types.h"

#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

const char *const kLcmStatusChannel = "IIWA_STATUS";
const char *const kLcmCommandChannel = "IIWA_COMMAND";
const int kNumJoints = 7;

class IiwaPlanRunner {
public:
  IiwaPlanRunner() : plan_number_(0), plan_(nullptr), new_plan_(nullptr) {
    tree_ = std::make_shared<RigidBodyTreed>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                            "iiwa14_primitive_collision.urdf"),
        multibody::joints::kFixed, tree_.get());
    DRAKE_DEMAND(kNumJoints == tree_->get_num_positions());
    DRAKE_DEMAND(kNumJoints == tree_->get_num_actuators());
    receiver_lcm_.subscribe(kLcmStatusChannel, &IiwaPlanRunner::HandleStatus, this);
    current_robot_state_.resize(kNumJoints * 2, 1);
    has_received_new_status_ = false;
    waiting_for_first_message_ = true;
  }
  ~IiwaPlanRunner() { Stop(); }

  void Start() {
    publish_thread_ = std::thread(&IiwaPlanRunner::PublishCommand, this);
    subscriber_thread_ = std::thread(&IiwaPlanRunner::ReceiveRobotStatus, this);
  }

  void Stop() {
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
    if (subscriber_thread_.joinable()) {
      subscriber_thread_.join();
    }
  }

  Eigen::VectorXd get_current_robot_state() {
    std::lock_guard<std::mutex> lock(robot_status_mutex_);
    return current_robot_state_;
  } // current robot state
  Eigen::VectorXd get_current_robot_position() {
    std::lock_guard<std::mutex> lock(robot_status_mutex_);
    return current_robot_state_.head(kNumJoints);
  } // current robot configuration
  Eigen::VectorXd get_current_robot_velocity() {
    std::lock_guard<std::mutex> lock(robot_status_mutex_);
    return current_robot_state_.tail(kNumJoints);
  } // current robot velocity

  std::shared_ptr<const RigidBodyTreed> get_tree() { return tree_; };

  // This method updates new_plan_ from a nullptr to point to a new PlanBase object.
  // At the beginning of every command publisher loop, PlanRunner checks whether
  // new_plan_ is a nullptr.
  // If new_plan_ is not a nullptr, it is moved to plan_(which will be executed
  // immediately), and becomes a nullptr again.
  void QueueNewPlan(std::unique_ptr<PlanBase> new_plan) {
    std::lock_guard<std::mutex> lock(robot_plan_mutex_);
    new_plan_ = std::move(new_plan);
  }

  void ReceiveRobotStatus() {
    // lock mutex when printing so that the text is not mangled (by printing in
    // another thread).
    robot_status_mutex_.lock();
    std::cout << "Robot status receiver thread starting on thread "
              << std::this_thread::get_id() << std::endl;
    robot_status_mutex_.unlock();

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == receiver_lcm_.handleTimeout(10) || waiting_for_first_message_) {
        // Print something here so users know no LCM message has been received.
      }
    }
  }

  // This method is used by the condition_variable to prevent spurious wakeups.
  bool has_received_new_status() { return has_received_new_status_; }

  void PublishCommand() {
    robot_status_mutex_.lock();
    std::cout << "Command publisher thread starting on thread "
              << std::this_thread::get_id() << std::endl;
    robot_status_mutex_.unlock();

    int64_t start_time_us = -1;
    int64_t cur_time_us = -1;

    // Allocate and initialize stuff used in the loop.
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);
    Eigen::VectorXd q_commanded(kNumJoints), v_commanded(kNumJoints);
    Eigen::VectorXd current_robot_state;

    while (true) {
      // Put the thread to sleep until a new iiwa_status message is received by the
      // subscriber thread.
      std::unique_lock<std::mutex> status_lock(robot_status_mutex_);
      cv_.wait(status_lock, std::bind(&IiwaPlanRunner::has_received_new_status, this));
      has_received_new_status_ = false;
      cur_time_us = cur_time_us_;
      current_robot_state = current_robot_state_;

      // Calling unlock is necessary because when cv_.wait() returns, this
      // thread acquires the mutex, preventing the receiver thread from
      // executing.
      status_lock.unlock();

      if (plan_number_ == 0) {
        // This block should only run once, right after the infinite while loop
        // starts.
        std::cout << "Generating first plan(holding current position)..."
                  << std::endl;
        new_plan_ = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(
            tree_, current_robot_state_.head(kNumJoints));
      }

      if (new_plan_) {
        robot_plan_mutex_.lock();
        plan_ = std::move(new_plan_);
        robot_plan_mutex_.unlock();

        plan_number_++;
        start_time_us = cur_time_us;
        is_cur_plan_terminated_ = false;
        std::cout << "Starting plan No. " << plan_number_ << std::endl;
      }

      cur_plan_time_s_ = static_cast<double>(cur_time_us - start_time_us) / 1e6;
      plan_->Step(current_robot_state, cur_plan_time_s_, &q_commanded,
                  &v_commanded);

      // Stop if commanded q is "too different" from current q.
//      Eigen::VectorXd dq = q_commanded - current_robot_state_.head(kNumJoints);
//      for (int i = 0; i < kNumJoints; i++) {
//        if (std::abs(dq[i]) > 0.1) {
//          is_cur_plan_terminated_ = true;
//          break;
//        }
//      }
//
//      if (is_cur_plan_terminated_) {
//        std::cout << "Difference between q_commanded and q too large."
//                     " Aborting plan "
//                  << cur_plan_number << " and starting a new blank plan."
//                  << std::endl;
//        std::lock_guard<std::mutex> lock(robot_plan_mutex_);
//        new_plan_ = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(
//            tree_, current_robot_state_.head(kNumJoints));
//        continue;
//      }

      // construct and publish iiwa_command
      iiwa_command.utime = cur_time_us;
      for (int i = 0; i < kNumJoints; i++) {
        iiwa_command.joint_position[i] = q_commanded(i);
      }
      publihser_lcm_.publish(kLcmCommandChannel, &iiwa_command);
    }
  }

  void MoveToJointPosition(Eigen::Ref<const Eigen::VectorXd> q_final,
                           double duration = 4) {
    Eigen::VectorXd q0 = get_current_robot_position();
    std::vector<double> times{0, duration};
    std::vector<Eigen::MatrixXd> knots;
    knots.push_back(q0);
    knots.push_back(q_final);

    std::unique_ptr<PlanBase> plan = std::make_unique<JointSpaceTrajectoryPlan>(
        tree_, PPType::FirstOrderHold(times, knots));
    QueueNewPlan(std::move(plan));
  }

private:
  void HandleStatus(const lcm::ReceiveBuffer *, const std::string &,
                    const lcmt_iiwa_status *status) {
    {
      std::lock_guard<std::mutex> status_lock(robot_status_mutex_);
      iiwa_status_ = *status;
      has_received_new_status_ = true;
      waiting_for_first_message_ = false;
      cur_time_us_ = iiwa_status_.utime;
      for (int i = 0; i < kNumJoints; i++) {
        current_robot_state_[i] = iiwa_status_.joint_position_measured[i];
        current_robot_state_[i + kNumJoints] =
            iiwa_status_.joint_velocity_estimated[i];
      }
    }
    cv_.notify_all();
  }

  // This is a leftover method from the old iiwa_plan_runner.
  // We need a way to stop current command but it has not been implemented.
  void HandleStop(const lcm::ReceiveBuffer *, const std::string &,
                  const robotlocomotion::robot_plan_t *) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  std::shared_ptr<RigidBodyTreed> tree_;
  int plan_number_{};
  std::unique_ptr<PlanBase> plan_;
  std::unique_ptr<PlanBase> new_plan_;

  double cur_plan_time_s_;
  bool is_cur_plan_terminated_;

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

  // shared
  lcmt_iiwa_status iiwa_status_;
  bool has_received_new_status_;
  Eigen::VectorXd current_robot_state_;
  int64_t cur_time_us_ = -1;

};

} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake