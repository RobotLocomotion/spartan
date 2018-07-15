#include <chrono>
#include <string>

#include <drake_robot_control/plan_runner.h>
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
                                 "drake_robot_control/config/"
                                 "iiwa_plan_runner_config.yaml";
  autoExpandEnvironmentVariables(config_file_name);

  auto runner = RobotPlanRunner::GetInstance(config_file_name);
  const int kNumJoints = runner->get_rigid_body_tree()->get_num_positions();

  runner->Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  VectorXd q0(kNumJoints), q1(kNumJoints), q2(kNumJoints);
  q0.setZero();
  q1 << 0.2793, //
      0.6824,   //
      -0.0456,   //
      -1.4918,  //
      0.0754,  //
      0.9042,   //
      0.5961;   //
  q2 << 0, M_PI / 3, 0, 0, 0, 0, 0;

  runner->MoveToJointPosition(q0, 4.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  runner->MoveToJointPosition(q1, 4.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  double dx, dy, dz, fx, fy, fz;
  Vector3d delta_x, force_xyz;
  Eigen::Isometry3d T_ee;
  Eigen::Vector3d rpy;
  runner->GetEePoseInWorldFrame(&T_ee, &rpy);
  cout << "ee position\n" << T_ee.translation() << endl;
  cout << "ee rpy\n" << rpy << endl;

  while (true) {
    cout << "\nplease enter world frame Cartesian command in the form of dx dy dz ..." << endl;
    dx = std::numeric_limits<double>::infinity();
    dy = dx;
    dz = dx;
    fx = 0;
    fy = 0;
    fz = 0;
    std::cin >> dx >> dy >> dz;
    std::cin >> fx >> fy >> fz;

    if(std::abs(dx) > 0.3 || std::abs(dy) > 0.3 || std::abs(dz) > 0.3 ||
        std::abs(fx) > 15 || std::abs(fy) > 15 || std::abs(fz) > 15) {
      cout << "command incomplete or too large..." << endl;
      continue;
    }
    delta_x << dx, dy, dz;
    force_xyz << fx, fy, fz;
    cout << "commanded movement: " << endl;
    cout << delta_x << endl;
    cout << "commanded force_xyz (in world frame): " << endl;
    cout << force_xyz << endl;

    runner->MoveRelativeToCurrentEeCartesianPosition(delta_x, force_xyz, 5.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));
    runner->GetEePoseInWorldFrame(&T_ee, &rpy);
    cout << "ee position\n" << T_ee.translation() << endl;
    cout << "ee rpy\n" << rpy << endl;
  }

  runner->MoveToJointPosition(q0);

  return 0;
}

} // namespace
} // namespace robot_plan_runner
} // namespace drake

int main() { return drake::robot_plan_runner::do_main(); }
