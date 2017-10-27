#include <memory>

#include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/util/drakeGeometryUtil.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(minimum_step_size, 0.0001,
              "Minimum simulation step size, in seconds.");
DEFINE_double(maximum_step_size, 0.01,
              "Maximum simulation step size, in seconds.");
DEFINE_double(visualization_period, 0.01,
              "Step time, in seconds, that visualization is published at.");
DEFINE_double(num_plates, 5, "# of plates.");
DEFINE_bool(
    fixed_step_mode, false,
    "Whether to use fixed step mode (of maximum_step_size) for simulation.");

// From DRAKE_RESOURCE_ROOT in build/install/share/drake
const char kRackUrdf[] =
    "../../../../models/raw_models/dish_rack_001/dish_rack_001.urdf";
const char kPlateUrdf[] =
    "../../../../models/raw_models/dish_001/dish_001.urdf";

using drake::manipulation::util::WorldSimTreeBuilder;
using drake::manipulation::util::ModelInstanceInfo;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::DrakeVisualizer;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::InputPortDescriptor;
using drake::systems::OutputPort;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;

static inline double randrange(double min, double max) {
  return (((double)rand()) / RAND_MAX) * (max - min) + min;
}

// Constructs a complete rigid body plant for the scene.
// TODO(gizatt) replace this with a functional scene constructor
// from an SDF, and maybe some kind of annotation from a YAML if necessary.
template <typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant() {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("rack", kRackUrdf);
  tree_builder->StoreModel("plate", kPlateUrdf);

  tree_builder->AddFixedModelInstance("rack", Eigen::Vector3d::Zero() /* xyz */,
                                      Eigen::Vector3d::Zero() /* rpy */);

  for (int i = 0; i < (int)FLAGS_num_plates; i++) {
    tree_builder->AddFloatingModelInstance(
        "plate", Eigen::Vector3d(randrange(0.0, 1.5), randrange(0.0, 1.5),
                                 randrange(0.5, 1.5)) /* xyz */,
        Eigen::Vector3d(randrange(-3.14, 3.14), randrange(-3.14, 3.14),
                        randrange(-3.14, 3.14)) /* rpy */);
  }
  tree_builder->AddGround();

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());

  return plant;
}

int DoMain() {
  DiagramBuilder<double> builder;

  std::unique_ptr<RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>();
  model_ptr->set_name("plant");

  auto model =
      builder.template AddSystem<RigidBodyPlant<double>>(std::move(model_ptr));
  model->set_name("plant");

  const RigidBodyTree<double>& tree = model->get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;

  DrakeVisualizer* visualizer = builder.AddSystem<DrakeVisualizer>(tree, &lcm);
  visualizer->set_name("visualizer");
  builder.Connect(model->state_output_port(), visualizer->get_input_port(0));
  visualizer->set_publish_period(FLAGS_visualization_period);

  auto sys = builder.Build();
  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  auto integrator = simulator.get_mutable_integrator();
  integrator->set_maximum_step_size(FLAGS_maximum_step_size);
  integrator->set_fixed_step_mode(FLAGS_fixed_step_mode);
  integrator->set_throw_on_minimum_step_size_violation(false);
  integrator->set_requested_minimum_step_size(FLAGS_minimum_step_size);
  simulator.set_publish_every_time_step(false);
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

int main(int argc, char* argv[]) {
  // ROS will remove the ROS-specific arguments
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  return DoMain();
}
