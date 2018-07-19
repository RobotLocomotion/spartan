#include <chrono>
#include <string>

#include <common_utils/system_utils.h>
#include <drake_robot_control/plan_runner.h>

// ROS
#include "ros/ros.h"

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

  int argc;
  char **argv;
  ros::init(argc, argv, "plan_runner");
  ros::NodeHandle nh("plan_runner"); // sets the node's namespace

  auto runner = RobotPlanRunner::GetInstance(nh, config_file_name);
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
  q2 << -0.1456, //
      -0.6498,   //
      0.1090,   //
      -1.5984,  //
      0.0794,  //
      1.5141,   //
      0.4069;   //

  runner->MoveToJointPosition(q2, 4.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  runner->MoveToJointPosition(q1, 4.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  double dx, dy, dz;
  Vector3d delta_x;
  Eigen::Isometry3d T_ee;
  Eigen::Vector3d rpy;
  runner->GetEePoseInWorldFrame(&T_ee, &rpy);
  cout << "ee position\n" << T_ee.translation() << endl;
  cout << "ee rpy\n" << rpy << endl;

  math::RollPitchYawd rpy_drake(M_PI/6, M_PI/2, 0);

  while (true) {
    cout << "\nplease enter world frame Cartesian command in the form of dx dy dz ..." << endl;
    dx = std::numeric_limits<double>::infinity();
    dy = dx;
    dz = dx;
    std::cin >> dx >> dy >> dz;

    if(std::abs(dx) > 0.3 || std::abs(dy) > 0.3 || std::abs(dz) > 0.3) {
      cout << "command incomplete or too large..." << endl;
      continue;
    }
    delta_x << dx, dy, dz;
    cout << "commanded movement: " << endl;
    cout << delta_x << endl;

    runner->MoveRelativeToCurrentEeCartesianPosition(delta_x, rpy_drake.ToRotationMatrix(), 5.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));
    runner->GetEePoseInWorldFrame(&T_ee, &rpy);
    cout << "ee position\n" << T_ee.translation() << endl;
    cout << "ee rpy\n" << rpy << endl;
    cout << "ee rotation matrix\n" << T_ee.linear() << endl;
    cout << "ee rotation matirx desired\n" << rpy_drake.ToMatrix3ViaRotationMatrix() << endl;
  }

  runner->MoveToJointPosition(q0);

  return 0;
}

} // namespace
} // namespace robot_plan_runner
} // namespace drake

int main() { return drake::robot_plan_runner::do_main(); }
