/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "bot_core/robot_state_t.hpp"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace spartan {
namespace control {
namespace {

using namespace drake;
const char* const kLcmStatusChannel = "EST_ROBOT_STATE";
const char* const kLcmCommandChannel = "ROBOT_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// tree is aliased
  explicit RobotPlanRunner(const RigidBodyTree<double>& tree)
      : tree_(tree), plan_number_(0) {
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    this->num_positions_ = tree_.get_num_positions();
  }

  void Run() {
    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    est_robot_state_.utime = cur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = this->num_positions_;
    iiwa_command.joint_position.resize(this->num_positions_, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(this->num_positions_, 0.);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || est_robot_state_.utime == -1) { }

      cur_time_us = est_robot_state_.utime;

      if (plan_) {
        if (plan_number_ != cur_plan_number) {
          std::cout << "Starting new plan." << std::endl;
          start_time_us = cur_time_us;
          cur_plan_number = plan_number_;
        }

        const double cur_traj_time_s =
            static_cast<double>(cur_time_us - start_time_us) / 1e6;
        const auto desired_next = plan_->value(cur_traj_time_s);

        iiwa_command.utime = est_robot_state_.utime;

        for (int joint = 0; joint < this->num_positions_; joint++) {
          iiwa_command.joint_position[joint] = desired_next(joint);
        }

        lcm_.publish(kLcmCommandChannel, &iiwa_command);
      }
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const bot_core::robot_state_t* status) {
    this->est_robot_state_ = *status;
  }

  void HandlePlan(const lcm::ReceiveBuffer*, const std::string&,
                  const robotlocomotion::robot_plan_t* plan) {
    std::cout << "New plan received." << std::endl;
    if (est_robot_state_.utime == -1) {
      std::cout << "Discarding plan, no status message received yet"
                << std::endl;
      return;
    }

    std::vector<Eigen::MatrixXd> knots(plan->num_states,
                                       Eigen::MatrixXd::Zero(this->num_positions_, 1));
    std::map<std::string, int> name_to_idx =
        tree_.computePositionNameToIndexMap();
    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];
      for (int j = 0; j < state.num_joints; ++j) {
        if (name_to_idx.count(state.joint_name[j]) == 0) {
          continue;
        }
        // Treat the matrix at knots[i] as a column vector.
        if (i == 0) {
          // Always start moving from the position which we're
          // currently commanding.
          DRAKE_DEMAND(est_robot_state_.utime != -1);
          knots[0](name_to_idx[state.joint_name[j]], 0) =
              est_robot_state_.joint_position[j];
        } else {
          knots[i](name_to_idx[state.joint_name[j]], 0) =
              state.joint_position[j];
        }
      }
    }

    for (int i = 0; i < plan->num_states; ++i) {
      std::cout << knots[i] << std::endl;
    }

    std::vector<double> input_time;
    for (int k = 0; k < static_cast<int>(plan->plan.size()); ++k) {
      input_time.push_back(plan->plan[k].utime / 1e6);
    }
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(this->num_positions_, 1);
    plan_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::Cubic(input_time, knots,
                                           knot_dot, knot_dot)));
    ++plan_number_;
  }

  lcm::LCM lcm_;
  const RigidBodyTree<double>& tree_;
  int num_positions_;
  int plan_number_{};
  std::unique_ptr<PiecewisePolynomialTrajectory> plan_;
  lcmt_iiwa_status iiwa_status_;
  bot_core::robot_state_t est_robot_state_;
};

int do_main() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  std::string spartan_source_dir = std::getenv("SPARTAN_SOURCE_DIR");
  std::string urdf_file = spartan_source_dir + "/models/ur10/ur10_description/ur10_robot_no_collision.urdf";
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(urdf_file,
      multibody::joints::kFixed, tree.get());

  RobotPlanRunner runner(*tree);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace control
}  // namespace spartan


int main() {
  return spartan::control::do_main();
}
