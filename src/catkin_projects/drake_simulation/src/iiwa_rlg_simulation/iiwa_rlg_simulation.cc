/// @file
///
/// Based on Drake's `examples/kuka_iiwa_arm/iiwa_wsg_simulation.cc`
/// example, but extended to allow simulation of a system with a
/// depth sensor, and to load an environment description from an SDF or
/// YAML or whatever I decide later.
///
/// Like the driver for the physical arm, this simulation communicates over LCM
/// using lcmt_iiwa_status and lcmt_iiwa_command messages for the arm, and the
/// lcmt_schunk_status and lcmt_schunk_command messages for the
/// gripper. It is intended to be a be a direct replacement for the
/// KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "bot_core/images_t.hpp"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
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
#include "drake/systems/sensors/rgbd_camera.h"
#include "drake/util/drakeGeometryUtil.h"

#include "ros/ros.h"
#include "wsg50_msgs/WSG_50_command.h"
#include "wsg50_msgs/WSG_50_state.h"

#include "drake_ros_systems/ros_publisher_system.h"
#include "drake_ros_systems/ros_subscriber_system.h"

#include "iiwa_common.h"
#include "iiwa_lcm.h"
#include "iiwa_world/iiwa_wsg_diagram_factory.h"
#include "image_driver/image_to_images_t.h"
#include "image_driver/rgbd_to_ros_pointcloud2_driver.h"
#include "schunk_wsg_ros/schunk_wsg_ros.h"
#include "schunk_wsg_ros/schunk_wsg_ros_controller.h"

#include "common_utils/system_utils.h"
#include "yaml-cpp/yaml.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(camera_update_period, 0.1, "Camera update period, in seconds.");
DEFINE_double(minimum_step_size, 0.0001,
              "Minimum simulation step size, in seconds.");
DEFINE_double(maximum_step_size, 0.01,
              "Maximum simulation step size, in seconds.");
DEFINE_double(
    rbt_timestep, 0.0,
    "RBT timestep (0.0 = continuous sim, positive = timestepping mode).");

DEFINE_bool(
    fixed_step_mode, false,
    "Whether to use fixed step mode (of maximum_step_size) for simulation.");

DEFINE_string(config, "", "Sim config filename (required).");

using drake::manipulation::util::WorldSimTreeBuilder;
using drake::manipulation::util::ModelInstanceInfo;
using drake::systems::sensors::RgbdCamera;
using drake::systems::sensors::RgbdCameraDiscrete;
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

using drake_ros_systems::RosPublisherSystem;
using drake_ros_systems::RosSubscriberSystem;

using namespace spartan::drake_simulation::iiwa_rlg_simulation;

constexpr char kCameraBaseFrameName[] = "camera_base_frame";
constexpr char kColorCameraFrameName[] = "color_camera_optical_frame";
constexpr char kDepthCameraFrameName[] = "depth_camera_optical_frame";
constexpr char kLabelCameraFrameName[] = "label_camera_optical_frame";

constexpr char kImageArrayLcmChannelName[] = "OPENNI_FRAME";
constexpr char kPoseLcmChannelName[] = "DRAKE_RGBD_CAMERA_POSE";

constexpr char kSchunkWsgCommandTopic[] = "/schunk_driver/schunk_wsg_command";
constexpr char kSchunkWsgStatusTopic[] = "/schunk_driver/schunk_wsg_status";
constexpr double kSchunkWsgStatusPeriod = 0.01;

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char* const kSchunkUrdf =
    "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

// Constructs a complete rigid body plant for the scene given a
// YAML scene configuration.
template <typename T>
std::unique_ptr<RigidBodyPlant<T>> BuildCombinedPlant(
    ModelInstanceInfo<T>* iiwa_instance, ModelInstanceInfo<T>* wsg_instance,
    std::vector<ModelInstanceInfo<T>>* object_instances, YAML::Node config) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.

  // Add a kuka (without a hand attached)
  tree_builder->StoreDrakeModel("iiwa", kIiwaUrdf);
  printf("Registered iiwa at %s\n", kIiwaUrdf);

  // Add the WSG hand
  tree_builder->StoreDrakeModel("wsg", kSchunkUrdf);
  printf("Registered schunk gripper at %s\n", kSchunkUrdf);

  // Adds config-requested models to the builder.
  for (const auto& node : config["models"]) {
    std::string full_path =
        expandEnvironmentVariables(node.second.as<std::string>());
    printf("Registering additional model %s:%s\n",
           node.first.as<std::string>().c_str(), full_path.c_str());
    tree_builder->StoreModel(node.first.as<std::string>(), full_path);
  }

  // Adds each model requested
  if (config["with_ground"] && config["with_ground"].as<bool>() == true) {
    tree_builder->AddGround();
  }

  // Add the IIWA at requested base position (default at origin)
  int id;
  Eigen::Vector3d robotBaseXyz(0, 0, 0);
  Eigen::Vector3d robotBaseRpy(0, 0, 0);
  if (config["robot"]) {
    std::vector<double> pose =
        config["robot"]["base_pose"].as<std::vector<double>>();
    Eigen::Vector3d xyz(pose[0], pose[1], pose[2]);
    Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);
    id = tree_builder->AddFixedModelInstance("iiwa", xyz, rpy);
    *iiwa_instance = tree_builder->get_model_info_for_instance(id);
  }

  // Add the schunk gripper on its end
  id = tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(id);

  // Add all requested objects
  for (const auto& node : config["instances"]) {
    std::vector<double> pose = node["q0"].as<std::vector<double>>();
    Eigen::Vector3d xyz(pose[0], pose[1], pose[2]);
    Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);
    if (node["fixed"].as<bool>())
      id = tree_builder->AddFixedModelInstance(node["model"].as<std::string>(),
                                               xyz, rpy);
    else
      id = tree_builder->AddFloatingModelInstance(
          node["model"].as<std::string>(), xyz, rpy);
    object_instances->push_back(tree_builder->get_model_info_for_instance(id));
  }

  auto plant = std::make_unique<RigidBodyPlant<T>>(tree_builder->Build(),
                                                   FLAGS_rbt_timestep);

  return plant;
}

void rosSpin() {
  while (ros::ok()) {
    ros::spinOnce();
    usleep(10);
  }
}

int DoMain(ros::NodeHandle& node_handle) {
  DiagramBuilder<double> builder;

  ModelInstanceInfo<double> iiwa_instance, wsg_instance;
  std::vector<ModelInstanceInfo<double>> object_instances;

  YAML::Node yaml_config = YAML::LoadFile(FLAGS_config);
  std::unique_ptr<RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &wsg_instance,
                                 &object_instances, yaml_config);
  model_ptr->set_name("plant");

  auto model =
      builder.template AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
          std::move(model_ptr),
          std::vector<ModelInstanceInfo<double>>({iiwa_instance}), 
          std::vector<ModelInstanceInfo<double>>({wsg_instance}), 
          std::vector<ModelInstanceInfo<double>>({}));
  model->set_name("plant_with_state_estimator");

  const RigidBodyTree<double>& tree = model->get_plant().get_rigid_body_tree();

  drake::lcm::DrakeLcm lcm;

  // Add camera
  struct CameraConfig {
    Eigen::Isometry3d pose;
    double fov_y{};
    double depth_range_near{};
    double depth_range_far{};
  };
  CameraConfig config;
  config.pose.setIdentity();
  config.pose.translation() = Eigen::Vector3d(0.00, 0., 0.05);
  config.pose.matrix().block<3, 3>(0, 0) =
      drake::math::rpy2rotmat(Eigen::Vector3d(0., 0.0, 0.));
  config.fov_y = M_PI_4;
  config.depth_range_near = 0.5;
  config.depth_range_far = 5.;

  // Make camera frame
  RigidBodyFrame<double> camera_frame(
      "iiwa_camera_frame", tree.FindBody("iiwa_link_ee"), config.pose);

  auto rgbd_camera = builder.AddSystem<RgbdCameraDiscrete>(
      std::make_unique<RgbdCamera>("rgbd_camera", tree, camera_frame,
                                   config.depth_range_near,
                                   config.depth_range_far, config.fov_y, true),
      FLAGS_camera_update_period);

  auto rgbd_to_ros_pointcloud2_driver =
      builder.template AddSystem<RgbdCameraRosDriver>(
          kColorCameraFrameName, kDepthCameraFrameName, "/camera", node_handle);
  rgbd_to_ros_pointcloud2_driver->set_name("image_publisher");
  rgbd_to_ros_pointcloud2_driver->set_publish_period(
      FLAGS_camera_update_period);

  DrakeVisualizer* visualizer = builder.AddSystem<DrakeVisualizer>(tree, &lcm);
  visualizer->set_name("visualizer");
  builder.Connect(model->get_output_port_plant_state(),
                  visualizer->get_input_port(0));
  visualizer->set_publish_period(kIiwaLcmStatusPeriod);

  // Create the command subscriber and status publisher.
  auto iiwa_command_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", &lcm));
  iiwa_command_sub->set_name("iiwa_command_subscriber");
  auto iiwa_command_receiver = builder.AddSystem<IiwaCommandReceiver>();
  iiwa_command_receiver->set_name("iwwa_command_receiver");

  auto iiwa_status_pub = builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_iiwa_status>("IIWA_STATUS", &lcm));
  iiwa_status_pub->set_name("iiwa_status_publisher");
  iiwa_status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  auto iiwa_status_sender = builder.AddSystem<IiwaStatusSender>();
  iiwa_status_sender->set_name("iiwa_status_sender");

  // TODO(siyuan): Connect this to kuka_planner runner once it generates
  // reference acceleration.
  auto iiwa_zero_acceleration_source =
      builder.template AddSystem<drake::systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Zero(7));
  iiwa_zero_acceleration_source->set_name("zero_acceleration");

  builder.Connect(iiwa_command_sub->get_output_port(0),
                  iiwa_command_receiver->get_input_port(0));
  builder.Connect(iiwa_command_receiver->get_output_port(0),
                  model->get_input_port_iiwa_state_command());
  builder.Connect(iiwa_zero_acceleration_source->get_output_port(),
                  model->get_input_port_iiwa_acceleration_command());

  builder.Connect(model->get_output_port_iiwa_state(),
                  iiwa_status_sender->get_state_input_port());
  builder.Connect(iiwa_command_receiver->get_output_port(0),
                  iiwa_status_sender->get_command_input_port());
  builder.Connect(iiwa_status_sender->get_output_port(0),
                  iiwa_status_pub->get_input_port(0));

  builder.Connect(model->get_output_port_plant_state(),
                  rgbd_camera->state_input_port());
  builder.Connect(rgbd_camera->color_image_output_port(),
                  rgbd_to_ros_pointcloud2_driver->color_image_input_port());
  builder.Connect(rgbd_camera->depth_image_output_port(),
                  rgbd_to_ros_pointcloud2_driver->depth_image_input_port());

  auto wsg_command_sub =
      builder.AddSystem(RosSubscriberSystem<wsg50_msgs::WSG_50_command>::Make(
          kSchunkWsgCommandTopic, &node_handle));
  wsg_command_sub->set_name("wsg_command_subscriber");
  auto wsg_controller = builder.AddSystem<SchunkWsgRosController>();

  auto wsg_status_pub =
      builder.AddSystem(RosPublisherSystem<wsg50_msgs::WSG_50_state>::Make(
          kSchunkWsgStatusTopic, &node_handle));
  wsg_status_pub->set_name("wsg_status_publisher");
  wsg_status_pub->set_publish_period(kSchunkWsgStatusPeriod);

  auto wsg_status_sender = builder.AddSystem<SchunkWsgRosStatusSender>(
      model->get_output_port_wsg_state().size(), 0, 1);
  wsg_status_sender->set_name("wsg_status_sender");

  builder.Connect(wsg_command_sub->get_output_port(0),
                  wsg_controller->get_command_input_port());
  builder.Connect(wsg_controller->get_output_port(0),
                  model->get_input_port_wsg_command());
  builder.Connect(model->get_output_port_wsg_state(),
                  wsg_status_sender->get_input_port(0));
  builder.Connect(model->get_output_port_wsg_state(),
                  wsg_controller->get_state_input_port());
  builder.Connect(*wsg_status_sender, *wsg_status_pub);

  auto iiwa_state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<bot_core::robot_state_t>(
          "EST_ROBOT_STATE", &lcm));
  iiwa_state_pub->set_name("iiwa_state_publisher");
  iiwa_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

  builder.Connect(model->get_output_port_iiwa_robot_state_msg(),
                  iiwa_state_pub->get_input_port(0));
  iiwa_state_pub->set_publish_period(kIiwaLcmStatusPeriod);

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

  std::thread spinner(rosSpin);
  simulator.StepTo(FLAGS_simulation_sec);
  spinner.join();

  return 0;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "iiwa_rlg_simulation");
  ros::NodeHandle node_handle;

  // ROS will remove the ROS-specific arguments
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  return DoMain(node_handle);
}