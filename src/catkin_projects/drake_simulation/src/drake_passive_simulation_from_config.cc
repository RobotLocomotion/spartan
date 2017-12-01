#include <memory>

#include <gflags/gflags.h>
#include "common_utils/system_utils.h"
#include "yaml-cpp/yaml.h"

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
DEFINE_double(target_realtime_rate, 1.0,
              "Target sim rate (as a fraction of real time).");
DEFINE_bool(
    fixed_step_mode, false,
    "Whether to use fixed step mode (of maximum_step_size) for simulation.");
DEFINE_string(config, "", "Sim config filename (required).");

DEFINE_double(us, 0.9, "The coefficient of static friction");
DEFINE_double(ud, 0.5, "The coefficient of dynamic friction");
DEFINE_double(stiffness, 10000, "The contact material stiffness");
DEFINE_double(dissipation, 2.0, "The contact material dissipation");
DEFINE_double(v_stiction_tolerance, 0.01,
              "The maximum slipping speed allowed during stiction");


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

// Constructs a complete rigid body plant for a scene
// from a configuration file.
template <typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant(YAML::Node config) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  for (const auto& node : config["models"]) {
    std::string full_path =
        expandEnvironmentVariables(node.second.as<std::string>());
    printf("Registering model %s:%s\n", node.first.as<std::string>().c_str(),
           full_path.c_str());
    tree_builder->StoreModel(node.first.as<std::string>(), full_path);
  }

  // Adds each model requested
  if (config["with_ground"] && config["with_ground"].as<bool>() == true) {
    tree_builder->AddGround();
  }

  for (const auto& node : config["instances"]) {
    std::vector<double> pose = node["q0"].as<std::vector<double>>();
    Eigen::Vector3d xyz(pose[0], pose[1], pose[2]);
    Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);
    if (node["fixed"].as<bool>())
      tree_builder->AddFixedModelInstance(node["model"].as<std::string>(), xyz,
                                          rpy);
    else
      tree_builder->AddFloatingModelInstance(node["model"].as<std::string>(),
                                             xyz, rpy);
  }

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build());

  return plant;
};

int DoMain() {
  DiagramBuilder<double> builder;

  YAML::Node config = YAML::LoadFile(FLAGS_config);
  std::unique_ptr<RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(config);
  model_ptr->set_name("plant");

  auto model =
      builder.template AddSystem<RigidBodyPlant<double>>(std::move(model_ptr));
  model->set_name("plant");

  // Command-line specified contact parameters.
  std::cout << "Contact properties:\n";
  std::cout << "\tStiffness:                " << FLAGS_stiffness << "\n";
  std::cout << "\tstatic friction:          " << FLAGS_us << "\n";
  std::cout << "\tdynamic friction:         " << FLAGS_ud << "\n";
  std::cout << "\tAllowed stiction speed:   " << FLAGS_v_stiction_tolerance
            << "\n";
  std::cout << "\tDissipation:              " << FLAGS_dissipation << "\n";
  model->set_normal_contact_parameters(FLAGS_stiffness, FLAGS_dissipation);
  model->set_friction_contact_parameters(FLAGS_us, FLAGS_ud,
                                         FLAGS_v_stiction_tolerance);

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
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
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
