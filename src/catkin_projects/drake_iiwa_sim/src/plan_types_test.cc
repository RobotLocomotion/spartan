#include "plan_types.h"
#include <stdexcept>

#include <drake/common/find_resource.h>
#include <drake/multibody/parsers/urdf_parser.h>
using std::cout;
using std::endl;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
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
  Eigen::VectorXd q_commanded, v_commanded;
  std::vector<double> t{0, 1, 10, 1e80};

  std::unique_ptr<Plan> plan;


  plan = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(tree, q);
  cout << "hello world!" << endl;
  for(auto & ti:t) {
    plan->Step(x, ti, &q_commanded, &v_commanded);
    cout << "ti:" << ti << endl;
    cout << "q\n" << q_commanded << endl;
    cout << "v\n" << v_commanded << endl;
  }

  std::unique_ptr<Plan> plan2;
  plan2 = std::move(plan);

  if(plan) {
    cout << "plan is not false." << endl;
  } else {
    cout << "plan is false." << endl;
  }

  return 0;
}
} // namespace
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake

int main() { return drake::examples::kuka_iiwa_arm::do_main(); }