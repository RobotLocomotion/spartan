#include <chrono>

#include "plan_runner.h"
#include "plan_types.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

int do_main() {
  IiwaPlanRunner runner;
  runner.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  VectorXd q0(kNumJoints), q1(kNumJoints);
  q0.setZero();
  q1 << 0.6826, //
      0.4082,   //
      0.3147,   //
      -0.9317,  //
      -0.5904,  //
      0.4316,   //
      0.5157; // 0, M_PI/3, 0, 0, 0, 0, 0;

  runner.MoveToJointPosition(q0, 2.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  runner.MoveToJointPosition(q1);
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  runner.MoveToJointPosition(q0);

  //  auto plan = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(runner.get_tree(),
  //  q0);
  //  runner.QueueNewPlan(std::move(plan));

  return 0;
}

} // namespace
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake

int main() { return drake::examples::kuka_iiwa_arm::do_main(); }