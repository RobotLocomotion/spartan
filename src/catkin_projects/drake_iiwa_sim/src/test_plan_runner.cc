#include <chrono>
#include <string>

#include <robot_plan_runner/plan_runner.h>
#include <common_utils/system_utils.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::cout;
using std::endl;

namespace drake {
namespace robot_plan_runner {
namespace {

int do_main() {
  std::string config_file_name = "${SPARTAN_SOURCE_DIR}/src/catkin_projects/"
                                 "drake_iiwa_sim/config/"
                                 "iiwa_plan_runner_config.yaml";
  autoExpandEnvironmentVariables(config_file_name);

  auto runner = RobotPlanRunner::GetInstance(config_file_name);
  const int kNumJoints = runner->get_rigid_body_tree()->get_num_positions();

  runner->Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  VectorXd q0(kNumJoints), q1(kNumJoints), q2(kNumJoints);
  q0.setZero();
  q1 << 0.6826, //
      0.4082,   //
      0.3147,   //
      -0.9317,  //
      -0.5904,  //
      0.4316,   //
      0.5157;   //
  q2 << 0, M_PI / 3, 0, 0, 0, 0, 0;

  // Trigger discarding current command.
  runner->MoveToJointPosition(q2, 0.001);
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  runner->MoveToJointPosition(q0, 5.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  runner->MoveToJointPosition(q1, 5.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  const Vector3d delta_x1(0.2, 0, 0);
  const Vector3d delta_x2(0, -0.2, 0);
  const Vector3d delta_x3(0, 0, -0.2);

  Eigen::Isometry3d T_ee;

  T_ee = runner->get_ee_pose_in_world_frame();
  cout << "EE_pose\n" << T_ee.matrix() << endl;
  runner->MoveRelativeToCurrentEeCartesianPosition(delta_x3, 5.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5500));
  T_ee = runner->get_ee_pose_in_world_frame();
  cout << "EE_pose\n" << T_ee.matrix() << endl;

  runner->MoveRelativeToCurrentEeCartesianPosition(delta_x1, 5.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5500));
  T_ee = runner->get_ee_pose_in_world_frame();
  cout << "EE_pose\n" << T_ee.matrix() << endl;

  runner->MoveRelativeToCurrentEeCartesianPosition(delta_x2, 5.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5500));
  T_ee = runner->get_ee_pose_in_world_frame();
  cout << "EE_pose\n" << T_ee.matrix() << endl;

  runner->MoveToJointPosition(q0);

  return 0;
}

} // namespace
} // namespace robot_plan_runner
} // namespace drake

int main() { return drake::robot_plan_runner::do_main(); }