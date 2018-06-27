#pragma once
#include "drake_iiwa_sim/iiwa_lcm.h"

#include <memory>
#include <string>

#include <drake/multibody/rigid_body_tree.h>
#include <drake/systems/framework/leaf_system.h>

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

class IiwaQpInverseDynamicsController : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaQpInverseDynamicsController)

  IiwaQpInverseDynamicsController(std::unique_ptr<RigidBodyTree<double>> tree,
                                  const Eigen::VectorXd& kp,
                                  const Eigen::VectorXd& kd,
                                  double control_period = 0.005);

  const systems::OutputPort<double>& get_output_port_torque_commanded() const {
    return this->get_output_port(idx_output_port_commanded_torque_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_estimated_state() const {
    return this->get_input_port(idx_input_port_estimated_state_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_state_reference() const {
    return this->get_input_port(idx_input_port_state_reference_);
  }

  const systems::InputPortDescriptor<double>& get_input_port_torque_reference() const {
    return this->get_input_port(idx_input_port_reference_torque_);
  }

  void CopyStateOut(const systems::Context<double>& context,
                    systems::BasicVector<double>* output) const {
    output->SetFromVector(context.get_discrete_state(0).CopyToVector());
  }

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  const double control_period_;  // in seconds
  const int nq_ = kIiwaArmNumJoints;
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  std::unique_ptr<RigidBodyTreed> tree_{nullptr};
  int idx_input_port_estimated_state_{-1};
  int idx_input_port_state_reference_{-1};
  int idx_input_port_reference_torque_{-1};
  int idx_output_port_commanded_torque_{-1};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
