#include <iostream>
#include <memory>

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

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char *const kLcmStatusChannel = "IIWA_STATUS";
const char *const kLcmCommandChannel = "IIWA_COMMAND";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;

class PlanRunner {
 public:
  /// tree is aliased
  explicit PlanRunner(const std::shared_ptr<const RigidBodyTreed> tree)
      : tree_(tree), plan_number_(0) {
    DRAKE_DEMAND(kNumJoints == tree_->get_num_positions());
    DRAKE_DEMAND(kNumJoints == tree_->get_num_actuators());
    lcm_.subscribe(kLcmStatusChannel, &PlanRunner::HandleStatus, this);
    plan_ = nullptr;
    // for testing only. creates a BlankPlan that cannot be changed.
  }

  void Run() {
    // 1. check if we have a Plan.
    // If there's no Plan, create a Plan to hold the current robot state after
    // receiving one message on IIWA_STATUS.
    // If there is a Plan, execute the Plan.

    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = cur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    VectorXd x(kNumJoints*2);
    VectorXd q_commanded(kNumJoints), v_commanded(kNumJoints);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) {
      }

      cur_time_us = iiwa_status_.utime;

      for (int i = 0; i < kNumJoints; i++) {
        x[i] = iiwa_status_.joint_position_measured[i];
        x[i+kNumJoints] = iiwa_status_.joint_velocity_estimated[i];
      }

      if (!plan_) {
        // This block should only run once after the infinite while loop starts.
        std::cout << "Starting PlanRunner..." << std::endl;
        plan_ = std::make_unique<Plan>();
        plan_ = JointSpaceTrajectoryPlan::MakeBlankPlan(tree_, x.head(kNumJoints));
        plan_number_++;
      }

      if (plan_number_ != cur_plan_number) {
        std::cout << "Starting new plan." << std::endl;
        start_time_us = cur_time_us;
        cur_plan_number = plan_number_;
      }

      const double cur_traj_time_s =
          static_cast<double>(cur_time_us - start_time_us) / 1e6;

      plan_->Step(x, cur_traj_time_s, &q_commanded, &v_commanded);

      iiwa_command.utime = iiwa_status_.utime;

      for (int i = 0; i < kNumJoints; i++) {
        iiwa_command.joint_position[i] = q_commanded(i);
      }

      lcm_.publish(kLcmCommandChannel, &iiwa_command);

    }
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
  const std::shared_ptr<const RigidBodyTreed> tree_;
  int plan_number_{};
  // std::unique_ptr<PiecewisePolynomial<double>> plan_;
  lcmt_iiwa_status iiwa_status_;

  // added by PT
  // This variable is false when PlanRunner is constructed. And is set true
  // after receiving the first IIWA_STATUS message and contructing an empty
  // Plan.
  std::unique_ptr<Plan> plan_;
};

int do_main() {
  auto tree = std::make_shared<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_primitive_collision.urdf"),
      multibody::joints::kFixed, tree.get());

  PlanRunner runner(tree);
  runner.Run();
  return 0;
}

} // namespace
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake

int main() { return drake::examples::kuka_iiwa_arm::do_main(); }