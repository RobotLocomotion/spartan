#include <drake_robot_control/joint_space_trajectory_plan.h>
#include <drake_robot_control/task_space_trajectory_plan.h>
#include <stdexcept>

#include <drake/common/find_resource.h>
#include <drake/multibody/parsers/urdf_parser.h>
using std::cout;
using std::endl;

namespace drake {
namespace robot_plan_runner {
namespace {

int do_main() {
  auto tree2 = std::make_shared<RigidBodyTreed>();
  std::shared_ptr<RigidBodyTreed> tree;
  tree = tree2;
  cout << "tree: " << tree.use_count() << endl;
  cout << "tree2: " << tree2.use_count() << endl;

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_primitive_collision.urdf"),
      multibody::joints::kFixed, tree.get());

  Eigen::VectorXd q(7);
  q << 0, 0, 0, 0, 0, 10, 100;

  Eigen::VectorXd x(14);
  Eigen::VectorXd tau_external(7);
  Eigen::VectorXd q_commanded, v_commanded, tau_commanded;
  std::vector<double> t{0, 1, 10, 1e80};

  std::unique_ptr<PlanBase> plan;


  plan = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(tree->Clone(), q);
  cout << "hello world!" << endl;
  for(auto & ti:t) {
    plan->Step(x, tau_external, ti, &q_commanded, &v_commanded, &tau_commanded);
    cout << "ti:" << ti << endl;
    cout << "q\n" << q_commanded << endl;
    cout << "v\n" << v_commanded << endl;
  }

  std::unique_ptr<PlanBase> plan2;
  plan2 = std::move(plan);

  if(plan) {
    cout << "plan is not false." << endl;
  } else {
    cout << "plan is false." << endl;
  }

  Eigen::Vector3d x_ee(0,0,0);
  Eigen::Vector3d rpy_ee(0,0,0);
  std::vector<double> times{0, 2};
  std::vector<Eigen::MatrixXd> knots;
  knots.push_back(x_ee);
  knots.push_back(x_ee);
  plan =
      std::make_unique<EndEffectorOriginTrajectoryPlan>(
          tree->Clone(), PPType::FirstOrderHold(times, knots), rpy_ee);
  for(auto & ti:t) {
    plan->Step(x, tau_external, ti, &q_commanded, &v_commanded, &tau_commanded);
    cout << "ti:" << ti << endl;
    cout << "q\n" << q_commanded << endl;
    cout << "v\n" << v_commanded << endl;
  }
  return 0;
}
} // namespace
} // namespace robot_plan_runner
} // namespace drake

int main() { return drake::robot_plan_runner::do_main(); }
