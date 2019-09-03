#include <limits>

#include <gflags/gflags.h>
#include "common_utils/system_utils.h"

#include "drake_iiwa_sim/kuka_schunk_station.h"
#include "drake_iiwa_sim/ros_rgbd_camera_publisher.h"
#include "drake_iiwa_sim/ros_scene_graph_visualizer.h"
#include "drake_iiwa_sim/schunk_wsg_ros_actionserver.h"

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"

#include "drake/multibody/plant/externally_applied_spatial_force.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "robot_msgs/ExternallyAppliedSpatialForce.h"


namespace drake {

using systems::Context;
using systems::VectorBase;

namespace multibody {

class BoxDisturber : public systems::LeafSystem<double> {
 public:
  explicit BoxDisturber(BodyIndex sugar_index, std::string topic) {
    this->DeclareAbstractOutputPort(
        "spatial_forces",
        &BoxDisturber::CalcSpatialForceGravityCompensation);

    this->sugar_index = sugar_index;

    this->pos = Vector3<double>::Zero();
    this->force = Vector3<double>::Zero();
    this->torque = Vector3<double>::Zero();

    //DeclarePeriodicPublishEvent(0.03, 0.0, &BoxDisturber::DoPeriodicPublish);

    sub = nh_.subscribe(topic, 1000, &BoxDisturber::chatterCallback, this);
  }

 private:
  void CalcSpatialForceGravityCompensation(
      const Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* output) const {

    const SpatialForce<double> F_L1q_W(this->torque, this->force);
    
    output->resize(1 /* number of forces */);
    (*output)[0].body_index = this->sugar_index;
    (*output)[0].p_BoBq_B = this->pos;
    (*output)[0].F_Bq_W = F_L1q_W;

  }

  // systems::EventStatus DoPeriodicPublish(
  //   const Context<double>& context) const {
    
  //   std::cout << "in periodic publish" << std::endl;

  //   ros::spinOnce();
  //   return systems::EventStatus::Succeeded();
  // }

  void chatterCallback(const robot_msgs::ExternallyAppliedSpatialForce msg)
  {
    pos(0) = msg.wrench_application_point.x;
    pos(1) = msg.wrench_application_point.y;
    pos(2) = msg.wrench_application_point.z;

    force(0) = msg.wrench.force.x;
    force(1) = msg.wrench.force.y;
    force(2) = msg.wrench.force.z;

    torque(0) = msg.wrench.torque.x;
    torque(1) = msg.wrench.torque.y;
    torque(2) = msg.wrench.torque.z;

    // std::cout << "\n\n";
    // std::cout << "got a spatial force" << std::endl;
    // std::cout << "force\n" << force << std::endl;

  }


  BodyIndex sugar_index;
  mutable ros::NodeHandle nh_;
  ros::Subscriber sub;
  Vector3<double> pos;
  Vector3<double> force;
  Vector3<double> torque;

};

} // namespace multibody
} // namespace drake



namespace drake_iiwa_sim {
namespace {

// Runs a simulation of the kuka schunk station, spoofing
// the setup used in Spartan by:
// Exposing the LCM command ports of the robot
// Exposing the ROS control ports of the Schunk gripper
// TODO: Spoofing camera info + images on appropriate camera channels

using namespace drake;
using namespace drake::examples;

using Eigen::VectorXd;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::Parser;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, std::numeric_limits<double>::infinity(),
              "Simulation duration.");
DEFINE_string(config, "", "Sim config filename (required).");

RigidTransform<double> load_tf_from_yaml(YAML::Node tf_yaml) {
  DRAKE_DEMAND(tf_yaml["quaternion"]);
  DRAKE_DEMAND(tf_yaml["quaternion"]["w"]);
  DRAKE_DEMAND(tf_yaml["quaternion"]["x"]);
  DRAKE_DEMAND(tf_yaml["quaternion"]["y"]);
  DRAKE_DEMAND(tf_yaml["quaternion"]["z"]);
  DRAKE_DEMAND(tf_yaml["translation"]);
  DRAKE_DEMAND(tf_yaml["translation"]["x"]);
  DRAKE_DEMAND(tf_yaml["translation"]["y"]);
  DRAKE_DEMAND(tf_yaml["translation"]["z"]);
  RotationMatrix<double> rotation(
      Quaternion<double>(tf_yaml["quaternion"]["w"].as<double>(),
                         tf_yaml["quaternion"]["x"].as<double>(),
                         tf_yaml["quaternion"]["y"].as<double>(),
                         tf_yaml["quaternion"]["z"].as<double>()));
  RigidTransform<double> tf(
      rotation, Eigen::Vector3d(tf_yaml["translation"]["x"].as<double>(),
                                tf_yaml["translation"]["y"].as<double>(),
                                tf_yaml["translation"]["z"].as<double>()));
  return tf;
}


int do_main(int argc, char* argv[]) {
  ros::init(argc, argv, "kuka_schunk_station_simulation");

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string station_config_file = expandEnvironmentVariables(FLAGS_config);
  printf("Loading file %s\n", station_config_file.c_str());
  YAML::Node station_config = YAML::LoadFile(station_config_file);

  systems::DiagramBuilder<double> builder;

  // Create the Kuka + Schunk.
  auto station = builder.AddSystem<KukaSchunkStation>(
      station_config, 0.002, IiwaCollisionModel::kPolytopeCollision);

  // Add a work table in front of the robot, and to its side.
  const double dz_table_top_robot_base = 0.736 + 0.057 / 2.;
  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  auto plant = &station->get_mutable_multibody_plant();
  Parser parser(plant);
  const auto table_front =
      parser.AddModelFromFile(table_sdf_path, "table_front");
  plant->WeldFrames(
      plant->world_frame(), plant->GetFrameByName("link", table_front),
      RigidTransform<double>(Eigen::Vector3d(0.75, 0, -dz_table_top_robot_base))
          .GetAsIsometry3());
  const auto table_left = parser.AddModelFromFile(table_sdf_path, "table_left");
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("link", table_left),
                    RigidTransform<double>(
                        Eigen::Vector3d(0.0, 0.8, -dz_table_top_robot_base))
                        .GetAsIsometry3());

  // TODO(gizatt) Merge into Schunk station, or its own class?
  // Assembly a list of initializations
  typedef struct {
    drake::multibody::ModelInstanceIndex model_instance;
    std::string body_name;
    Eigen::Isometry3d tf;
  } ObjectInitializationInfo;

  std::vector<ObjectInitializationInfo> initializations_to_do;
  int k = 0;
  for (const auto& node : station_config["instances"]) {
    const auto pose = node["q0"].as<std::vector<double>>();
    Eigen::Vector3d xyz(pose[0], pose[1], pose[2]);
    Eigen::Vector3d rpy(pose[3], pose[4], pose[5]);
    Eigen::Isometry3d object_tf(
        RigidTransform<double>(RollPitchYaw<double>(rpy), xyz)
            .GetAsIsometry3());

    const auto object_class = node["model"].as<std::string>();
    auto object_class_node = station_config["models"][object_class];
    DRAKE_DEMAND(object_class_node);
    std::string full_path =
        expandEnvironmentVariables(object_class_node.as<std::string>());
    // TODO: replace with unique name
    std::stringstream model_name;
    model_name << node["model"].as<std::string>() << "_" << k++;
    std::string body_name = node["body_name"].as<std::string>();
    drake::multibody::ModelInstanceIndex model_instance;
    model_instance = parser.AddModelFromFile(full_path, model_name.str());

    if (node["fixed"].as<bool>()) {
      // Cludgy, but default behavior in AddModelFromFile is to
      // make a frame at the root of the added model with the same name
      // as the added model.
      plant->WeldFrames(
          plant->world_frame(),
          plant->GetBodyByName(body_name, model_instance).body_frame(),
          object_tf);
    } else {
      initializations_to_do.push_back(
          ObjectInitializationInfo({model_instance, body_name, object_tf}));
    }
  }


  station->Finalize();


  // TODO(gizatt) Merge this into the Schunk Station, or its own
  // class?
  {
    auto render_scene_graph =
        builder.template AddSystem<drake::geometry::dev::SceneGraph>(
            station->get_scene_graph());
    builder.Connect(station->GetOutputPort("geometry_poses"),
                    render_scene_graph->get_source_pose_port(
                        plant->get_source_id().value()));

    if (station_config["cameras"]) {
      
      for (const auto camera_config : station_config["cameras"]) {

        std::cout << camera_config["name"] << std::endl;


        DRAKE_DEMAND(camera_config["name"]);
        DRAKE_DEMAND(camera_config["channel"]);
        DRAKE_DEMAND(camera_config["config_base_dir"]);
        YAML::Node camera_extrinsics_yaml =
            YAML::LoadFile(expandEnvironmentVariables(
                camera_config["config_base_dir"].as<std::string>() +
                "/camera_info.yaml"));
        YAML::Node rgb_camera_info_yaml =
            YAML::LoadFile(expandEnvironmentVariables(
                camera_config["config_base_dir"].as<std::string>() +
                "/rgb_camera_info.yaml"));
        YAML::Node depth_camera_info_yaml =
            YAML::LoadFile(expandEnvironmentVariables(
                camera_config["config_base_dir"].as<std::string>() +
                "/depth_camera_info.yaml"));

        auto camera_name = camera_config["name"].as<std::string>();
        // TODO(gizatt) Load this from the instrinsics yamls
        // specified above. Right now, loading defaults for a Carmine
        // (640x480, 54 deg horizontal FOV, .3-3.0 meter range.)
        // These don't *have* to match is downstream users only
        // use the point cloud -- the rectification + registration driver
        // consumes the camera_info messages that we construct based on
        // the parameters set here, not based on the referred-to
        // camera calibration.
        drake::geometry::dev::render::DepthCameraProperties camera_properties(
            640, 480, 54 * 3.1412 / 180., geometry::dev::render::Fidelity::kLow,
            0.3, 3.0);

        // I'm currently loading in the model + body name for the camera
        // mount directly from the sim configuration, as they're specified
        // in a non-unique way in the Spartan camera calibration files
        // (i.e. only frame name, no model, and assuming we're using Spartan
        // version of the WSG URDF...)
        const auto body_node_index =
            plant
                ->GetBodyByName(
                    camera_config["mounting_body_name"].as<std::string>(),
                    plant->GetModelInstanceByName(
                        camera_config["mounting_model_name"].as<std::string>()))
                .index();
        const auto depth_camera_frame_id =
            plant->GetBodyFrameIdOrThrow(body_node_index);

        auto camera_origin_correction = RigidTransform<double>::Identity();
        if (camera_config["mounting_correction"]) {
          camera_origin_correction =
              load_tf_from_yaml(camera_config["mounting_correction"]);
        }

        auto camera =
            builder
                .template AddSystem<drake::systems::sensors::dev::RgbdCamera>(
                    camera_name, depth_camera_frame_id,
                    camera_origin_correction.GetAsIsometry3(),
                    camera_properties, false);

        RigidTransform<double> depth_camera_tf = load_tf_from_yaml(
            camera_extrinsics_yaml["depth"]["extrinsics"]
                                  ["transform_to_reference_link"]);
        RigidTransform<double> color_camera_tf = load_tf_from_yaml(
            camera_extrinsics_yaml["rgb"]["extrinsics"]
                                  ["transform_to_reference_link"]);

        // pete: this is a slow way to move the cameras and doesn't match ROS
        // color_camera_tf = RigidTransform<double>(
        //                      RollPitchYaw<double>(0.0, 0.0, 0.0),
        //                      Eigen::Vector3d(0.0, 0.0, i-1.0));

        camera->set_depth_camera_optical_pose(depth_camera_tf.GetAsIsometry3());

        camera->set_color_camera_optical_pose(color_camera_tf.GetAsIsometry3());

        builder.Connect(render_scene_graph->get_query_output_port(),
                        camera->query_object_input_port());

        auto camera_publisher =
            builder.template AddSystem<RosRgbdCameraPublisher>(
                *camera, camera_name, 0.0333);
        builder.Connect(camera->color_image_output_port(),
                        camera_publisher->color_image_input_port());
        builder.Connect(camera->depth_image_output_port(),
                        camera_publisher->depth_image_input_port());
        builder.Connect(camera->label_image_output_port(),
                        camera_publisher->label_image_input_port());
        builder.Connect(camera->camera_base_pose_output_port(),
                        camera_publisher->camera_base_pose_input_port());
      }
    }
  }

  // Visualizers
  geometry::ConnectDrakeVisualizer(&builder, station->get_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));
  auto ros_visualizer =
      builder.AddSystem<RosSceneGraphVisualizer>(station->get_scene_graph());
  builder.Connect(station->GetOutputPort("pose_bundle"),
                  ros_visualizer->get_pose_bundle_input_port());

  drake::lcm::DrakeLcm lcm;
  lcm.StartReceiveThread();

   // TODO(russt): IiwaCommandReceiver should output positions, not
  // state.  (We are adding delay twice in this current implementation).
  auto iiwa_command_subscriber = builder.AddSystem(
      kuka_iiwa_arm::MakeIiwaCommandLcmSubscriberSystem(
          kuka_iiwa_arm::kIiwaArmNumJoints, "IIWA_COMMAND", &lcm));
  auto iiwa_command = builder.AddSystem<kuka_iiwa_arm::IiwaCommandReceiver>();
  builder.Connect(iiwa_command_subscriber->get_output_port(),
                  iiwa_command->GetInputPort("command_message"));

  // Pull the positions out of the state.
  auto demux = builder.AddSystem<systems::Demultiplexer>(14, 7);
  builder.Connect(iiwa_command->get_commanded_state_output_port(),
                  demux->get_input_port(0));
  builder.Connect(demux->get_output_port(0),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(iiwa_command->get_commanded_torque_output_port(),
                  station->GetInputPort("iiwa_feedforward_torque"));

  auto iiwa_status = builder.AddSystem<kuka_iiwa_arm::IiwaStatusSender>();
  // The IiwaStatusSender input port wants size 14, but only uses the first 7.
  // TODO(russt): Consider cleaning up the IiwaStatusSender.
  auto zero_padding =
      builder.AddSystem<systems::MatrixGain>(Eigen::MatrixXd::Identity(14, 7));
  builder.Connect(station->GetOutputPort("iiwa_position_commanded"),
                  zero_padding->get_input_port());
  builder.Connect(zero_padding->get_output_port(),
                  iiwa_status->get_command_input_port());
  builder.Connect(station->GetOutputPort("iiwa_state_estimated"),
                  iiwa_status->get_state_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_commanded"),
                  iiwa_status->get_commanded_torque_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_measured"),
                  iiwa_status->get_measured_torque_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                  iiwa_status->get_external_torque_input_port());
  auto iiwa_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", &lcm, 0.005 /* publish period */));
  builder.Connect(iiwa_status->get_output_port(0),
                  iiwa_status_publisher->get_input_port());

  auto wsg_ros_actionserver = builder.AddSystem<SchunkWsgActionServer>(
      "/wsg50_driver/wsg50/gripper_control/", "/wsg50_driver/wsg50/status");
  builder.Connect(wsg_ros_actionserver->get_position_output_port(),
                  station->GetInputPort("wsg_position"));
  builder.Connect(wsg_ros_actionserver->get_force_limit_output_port(),
                  station->GetInputPort("wsg_force_limit"));
  builder.Connect(station->GetOutputPort("wsg_state_measured"),
                  wsg_ros_actionserver->get_measured_state_input_port());
  builder.Connect(station->GetOutputPort("wsg_force_measured"),
                  wsg_ros_actionserver->get_measured_force_input_port());

  // Initialize object poses uses initialization list
  // generated when loading them in.
  
  // add ExternallyAppliedSpatial force for sugar box and plate
  // multibody::BodyIndex sugar_index;
  multibody::BodyIndex body_index;
  int counter = 0;
  std::string topic;
  for (const auto& initialization : initializations_to_do) {

        const auto& obj = plant->GetBodyByName(initialization.body_name,
                           initialization.model_instance);

        // if we found sugar box or plate, then add ExternallyAppliedSpatialForce
        if (initialization.body_name == "base_link_sugar"){

          topic = "/box_rotate";
          body_index = obj.index();
          counter += 1;
        }
        // if we found sugar box or plate, then add ExternallyAppliedSpatialForce
        if (initialization.body_name == "ikea_dinera_plate_8in"){

          topic = "/plate_external_wrench";
          body_index = obj.index();
          counter += 1;
        }

  }

  if (counter > 1){
    throw std::runtime_error("More than one object you are trying to apply an ExternalSpatialForce to, support at most one");
  }

  if (counter == 1){
    auto disturber =
    builder.AddSystem<multibody::BoxDisturber>(body_index, topic);

    // auto plant_something = builder.AddSystem<>();

    disturber->get_output_port(0);
    station->GetInputPort("applied_spatial_force");
    //&station->get_applied_spatial_force_input_port();
    std::cout << "could call these ports " << std::endl;

    //Connect the system to the MBP.
    builder.Connect(
        disturber->get_output_port(0),
        station->GetInputPort("applied_spatial_force"));
  }
 
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();
  auto& station_context =
      diagram->GetMutableSubsystemContext(*station, &context);

  // Set initial conditions for the IIWA:
  VectorXd q0(7);
  // Has good view of the table.
  q0 << -8.4, -37.3, 6.3, -91.5, 4.7, 87.4, 23.4;
  q0 *= 3.14 / 180.;

  iiwa_command->set_initial_position(
      &diagram->GetMutableSubsystemContext(*iiwa_command, &context), q0);

  station->SetIiwaPosition(&station_context, q0);
  const VectorXd qdot0 = VectorXd::Zero(7);
  station->SetIiwaVelocity(&station_context, qdot0);



  // Initialize object poses uses initialization list
  // generated when loading them in.
  for (const auto& initialization : initializations_to_do) {
    plant->SetFreeBodyPose(
        &station->GetMutableSubsystemContext(*plant, &station_context),
        plant->GetBodyByName(initialization.body_name,
                             initialization.model_instance),
        initialization.tf);
  }

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.set_publish_every_time_step(false);
  simulator.Initialize();
  simulator.StepTo(FLAGS_duration);

  return 0;
}

}  // namespace
}  // namespace drake_iiwa_sim

int main(int argc, char* argv[]) { return drake_iiwa_sim::do_main(argc, argv); }
