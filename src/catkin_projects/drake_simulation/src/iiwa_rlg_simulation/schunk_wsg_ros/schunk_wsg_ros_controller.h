#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"

namespace spartan {
namespace drake_simulation {
namespace iiwa_rlg_simulation {

using namespace drake;

/// This class implements a controller for a Schunk WSG gripper.  It
/// has two input ports which receive Schunk WSG50 ROS messages
/// and the current state, and an output port which emits the target
/// force for the actuated finger.  The internal implementation
/// consists of a PID controller (which controls the target position
/// from the command message) combined with a saturation block (which
/// applies the force control from the command message).
class SchunkWsgRosController : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SchunkWsgRosController)
  SchunkWsgRosController();

  const systems::InputPortDescriptor<double>& get_command_input_port() const {
    return this->get_input_port(command_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

 private:
  int command_input_port_{};
  int state_input_port_{};
};
}
}
}
