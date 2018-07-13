#include <drake_iiwa_sim/iiwa_qp_inverse_dynamics_controller.h>

#include <drake/multibody/kinematics_cache.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/systems/framework/basic_vector.h>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

typedef Eigen::Matrix<double, 7, 1> Vector7d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;

IiwaQpInverseDynamicsController::IiwaQpInverseDynamicsController(
    std::unique_ptr<RigidBodyTree<double>> tree, const VectorX<double> &kp,
    const VectorX<double> &kd, double control_period)
    : control_period_(control_period) {
  // sanity check
  DRAKE_DEMAND(nq_ == tree->get_num_positions());
  DRAKE_DEMAND(nq_ == kp.size());
  DRAKE_DEMAND(nq_ == kd.size());

  // copy stuff.
  tree_ = std::move(tree);
  kp_ = kp;
  kd_ = kd;

  // declare ports and states.
  this->DeclarePeriodicDiscreteUpdate(control_period_);
  // estimated state input port
  idx_input_port_estimated_state_ =
      this->DeclareVectorInputPort(systems::BasicVector<double>(2 * nq_))
          .get_index();
  // reference state input port
  idx_input_port_state_reference_ =
      this->DeclareVectorInputPort(systems::BasicVector<double>(2 * nq_))
          .get_index();
  // reference torque input port
  idx_input_port_reference_torque_ =
      this->DeclareVectorInputPort(systems::BasicVector<double>(nq_))
          .get_index();
  // commanded torque output port
  idx_output_port_commanded_torque_ =
      this->DeclareVectorOutputPort(
              systems::BasicVector<double>(nq_),
              &IiwaQpInverseDynamicsController::CopyStateOut)
          .get_index();
  // the system's state is the commanded torque sent to rigid body plant.
  this->DeclareDiscreteState(nq_);
}

void IiwaQpInverseDynamicsController::DoCalcDiscreteVariableUpdates(
    const systems::Context<double> &context,
    const std::vector<const systems::DiscreteUpdateEvent<double> *> &,
    systems::DiscreteValues<double> *discrete_state) const {
  // discrete state is the output (robot torque)
  const VectorXd x =
      (*this->EvalVectorInput(context,
                              get_input_port_estimated_state().get_index()))
          .CopyToVector();
  const VectorXd x_ref =
      (*this->EvalVectorInput(context,
                              get_input_port_state_reference().get_index()))
          .CopyToVector();
  const VectorXd tau_ref =
      (*this->EvalVectorInput(context,
                              get_input_port_torque_reference().get_index()))
          .CopyToVector();

  const Vector7d q = x.head(nq_);
  const Vector7d v = x.tail(nq_);
  const Vector7d q_ref = x_ref.head(nq_);
  const Vector7d v_ref = x_ref.tail(nq_);

  // create tree alias, so that clion doesn't complain about unique pointers.
  const RigidBodyTreed &tree = *tree_;
  KinematicsCache<double> cache = tree.CreateKinematicsCache();
  cache.initialize(q, v);
  tree.doKinematics(cache, true);

  MatrixXd H = tree.massMatrix(cache);
  RigidBodyTree<double>::BodyToWrenchMap external_wrenches;

  // desired acceleration
  Vector7d err_q = q_ref - q;
  Vector7d err_v = v_ref - v;
  VectorXd vd_d = kp_.array() * err_q.array() + kd_.array() * err_v.array();
  Vector7d tau = tree.inverseDynamics(cache, external_wrenches, vd_d);

  // const int idx_base_ = tree.FindBodyIndex("base");
  // const int idx_ee_ = tree.FindBodyIndex("iiwa_link_ee");

  discrete_state->get_mutable_vector().SetFromVector(tau);
}

} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake
