#pragma once

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
    lcm_.subscribe(kLcmStatusChannel, &IiwaPlanRunner::HandleStatus, this);
    x_.resize(kNumJoints * 2, 1);
  }
  ~IiwaPlanRunner() { Stop(); }

  void Start() {
    publish_thread_ = std::thread(&IiwaPlanRunner::PublishCommand, this);
  }

  void Stop() {
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
  }

  bool is_cur_plan_finished() {
    return (cur_plan_time_s_ >plan_->duration()) || is_cur_plan_terminated_;
  }

  Eigen::VectorXd get_current_x() { return x_; } // current robot state
  Eigen::VectorXd get_current_q() {
    return x_.head(kNumJoints);
  } // current robot configuration
  Eigen::VectorXd get_current_v() {
    return x_.tail(kNumJoints);
  } // current robot velocity

  std::shared_ptr<const RigidBodyTreed> get_tree() { return tree_; };

  void QueueNewPlan(std::unique_ptr<Plan> new_plan) {
    std::lock_guard<std::mutex> lock(mutex_);
    new_plan_ = std::move(new_plan);
  }

  void PublishCommand() {
    std::cout << "Command publisher thread starting on thread "
              << std::this_thread::get_id() << std::endl;

    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = cur_time_us;

    // Allocate and initialize stuff used in the loop.
    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);
    Eigen::VectorXd q_commanded(kNumJoints), v_commanded(kNumJoints);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
      }

      cur_time_us = iiwa_status_.utime;

      for (int i = 0; i < kNumJoints; i++) {
        x_[i] = iiwa_status_.joint_position_measured[i];
        x_[i + kNumJoints] = iiwa_status_.joint_velocity_estimated[i];
      }

      if (plan_number_ == 0) {
        // This block should only run once, right after the infinite while loop starts.
        std::cout << "Generating first plan(holding current position)..."
                  << std::endl;
        new_plan_ =
            JointSpaceTrajectoryPlan::MakeBlankPlan(tree_, x_.head(kNumJoints));
      }

      if (new_plan_) {
        mutex_.lock();
        plan_ = std::move(new_plan_);
        mutex_.unlock();

        plan_number_++;
        is_cur_plan_terminated_ = false;
        start_time_us = cur_time_us;
        cur_plan_number = plan_number_;
        std::cout << "Starting plan No. " << cur_plan_number << std::endl;
      }

      cur_plan_time_s_ =
          static_cast<double>(cur_time_us - start_time_us) / 1e6;

      plan_->Step(x_, cur_plan_time_s_, &q_commanded, &v_commanded);

      // Stop if commanded q is "too different" from current q.
      Eigen::VectorXd dq = q_commanded - x_.head(kNumJoints);
      for (int i = 0; i < kNumJoints; i++) {
        if (std::abs(dq[i]) > 0.1) {
          is_cur_plan_terminated_ = true;
          break;
        }
      }

      if (is_cur_plan_terminated_) {
        std::cout << "Difference between q_commanded and q too large. Aborting "
                     "plan "
                  << cur_plan_number << " and starting a new blank plan."
                  << std::endl;
        std::lock_guard<std::mutex> lock(mutex_);
        new_plan_ =
            JointSpaceTrajectoryPlan::MakeBlankPlan(tree_, x_.head(kNumJoints));
        continue;
      }

      // construct and publish iiwa_command
      iiwa_command.utime = iiwa_status_.utime;
      for (int i = 0; i < kNumJoints; i++) {
        iiwa_command.joint_position[i] = q_commanded(i);
      }
      lcm_.publish(kLcmCommandChannel, &iiwa_command);
    }
  }

  void MoveToJointPosition(Eigen::Ref<const Eigen::VectorXd> q_final,
                           double duration = 4) {
    Eigen::VectorXd q0 = get_current_q();
    std::vector<double> times{0, duration};
    std::vector<Eigen::MatrixXd> knots;
    knots.push_back(q0);
    knots.push_back(q_final);

    std::unique_ptr<Plan> plan = std::make_unique<JointSpaceTrajectoryPlan>(
        tree_, PPType::FirstOrderHold(times, knots));
    QueueNewPlan(std::move(plan));
  }

private:
  void HandleStatus(const lcm::ReceiveBuffer *, const std::string &,
                    const lcmt_iiwa_status *status) {
    iiwa_status_ = *status;
  }

  void HandleStop(const lcm::ReceiveBuffer *, const std::string &,
                  const robotlocomotion::robot_plan_t *) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  lcm::LCM lcm_;
  std::shared_ptr<RigidBodyTreed> tree_;
  int plan_number_{};
  lcmt_iiwa_status iiwa_status_;
  std::unique_ptr<Plan> plan_;
  std::unique_ptr<Plan> new_plan_;
  std::mutex mutex_;
  std::thread publish_thread_;
  Eigen::VectorXd x_; // current robot state
  double cur_plan_time_s_;
  bool is_cur_plan_terminated_;
};

} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake