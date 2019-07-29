#include "drake_iiwa_sim/kuka_schunk_station.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake_iiwa_sim {

using namespace drake;
using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::Joint;
using multibody::ModelInstanceIndex;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::MultibodyPlant;
using multibody::Parser;

const int kNumDofIiwa = 7;

// TODO(amcastro-tri): Refactor this into schunk_wsg directory, and cover it
// with a unit test.  Potentially tighten the tolerance in
// station_simulation_test.
SpatialInertia<double> MakeCompositeGripperInertia(
    const std::string& wsg_sdf_path) {
  MultibodyPlant<double> plant;
  Parser parser(&plant);
  parser.AddModelFromFile(wsg_sdf_path);
  plant.Finalize();
  const auto& gripper_body = plant.GetRigidBodyByName("body");
  const auto& left_finger = plant.GetRigidBodyByName("left_finger");
  const auto& right_finger = plant.GetRigidBodyByName("right_finger");
  const auto& left_slider = plant.GetJointByName("left_finger_sliding_joint");
  const auto& right_slider = plant.GetJointByName("right_finger_sliding_joint");
  const SpatialInertia<double>& M_GGo_G =
      gripper_body.default_spatial_inertia();
  const SpatialInertia<double>& M_LLo_L = left_finger.default_spatial_inertia();
  const SpatialInertia<double>& M_RRo_R =
      right_finger.default_spatial_inertia();
  auto CalcFingerPoseInGripperFrame = [](const Joint<double>& slider) {
    // Pose of the joint's parent frame P (attached on gripper body G) in the
    // frame of the gripper G.
    const RigidTransform<double> X_GP(
        slider.frame_on_parent().GetFixedPoseInBodyFrame());
    // Pose of the joint's child frame C (attached on the slider's finger body)
    // in the frame of the slider's finger F.
    const RigidTransform<double> X_FC(
        slider.frame_on_child().GetFixedPoseInBodyFrame());
    // When the slider's translational dof is zero, then P coincides with C.
    // Therefore:
    const RigidTransform<double> X_GF = X_GP * X_FC.inverse();
    return X_GF;
  };
  // Pose of left finger L in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GL(CalcFingerPoseInGripperFrame(left_slider));
  // Pose of right finger R in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GR(CalcFingerPoseInGripperFrame(right_slider));
  // Helper to compute the spatial inertia of a finger F in about the gripper's
  // origin Go, expressed in G.
  auto CalcFingerSpatialInertiaInGripperFrame = [](
      const SpatialInertia<double>& M_FFo_F,
      const RigidTransform<double>& X_GF) {
    const auto M_FFo_G = M_FFo_F.ReExpress(X_GF.rotation());
    const auto p_FoGo_G = -X_GF.translation();
    const auto M_FGo_G = M_FFo_G.Shift(p_FoGo_G);
    return M_FGo_G;
  };
  // Shift and re-express in G frame the finger's spatial inertias.
  const auto M_LGo_G = CalcFingerSpatialInertiaInGripperFrame(M_LLo_L, X_GL);
  const auto M_RGo_G = CalcFingerSpatialInertiaInGripperFrame(M_RRo_R, X_GR);
  // With everything about the same point Go and expressed in the same frame G,
  // proceed to compose into composite body C:
  // TODO(amcastro-tri): Implement operator+() in SpatialInertia.
  SpatialInertia<double> M_CGo_G = M_GGo_G;
  M_CGo_G += M_LGo_G;
  M_CGo_G += M_RGo_G;
  return M_CGo_G;
}

template <typename T>
KukaSchunkStation<T>::KukaSchunkStation(const YAML::Node& station_config, 
                                        double time_step,
                                        IiwaCollisionModel collision_model)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      owned_controller_plant_(std::make_unique<MultibodyPlant<T>>()) {
  station_config_ = station_config;
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);

  // Add the robot mounting table.
  const double dx_table_center_to_robot_base = 0.0;
  const double dz_table_top_robot_base = 0.736 + 0.057 / 2.;
  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf");
  Parser parser(plant_);
  const auto table =
      parser.AddModelFromFile(table_sdf_path, "table");
  plant_->WeldFrames(
      plant_->world_frame(), plant_->GetFrameByName("link", table),
      RigidTransform<double>(
          Vector3d(dx_table_center_to_robot_base, 0, -dz_table_top_robot_base))
          .GetAsIsometry3());

  if (station_config_["with_ground"] &&
      station_config_["with_ground"].as<bool>() == true) {
    // Add big (but non-infinite) flat floor.
    const double dz_floor = 2.0;
    drake::geometry::Box floor_shape(20., 20., dz_floor);
    auto floor_pose = RigidTransform<double>(
      Vector3d(0., 0., -dz_table_top_robot_base - dz_floor/2.)).GetAsIsometry3();
    
    /* Don't draw floor, it's really ugly in Rviz.
    plant_->RegisterVisualGeometry(
      plant_->world_body(), floor_pose, floor_shape, "floor_visual",
      drake::geometry::VisualMaterial({0.2, 0.4, 0.2, 1.0}));
    */
    plant_->RegisterCollisionGeometry(
      plant_->world_body(), floor_pose, floor_shape, "floor_collision",
      drake::multibody::multibody_plant::CoulombFriction<double>(0.9, 0.7));
  }

  // Add the Kuka IIWA.
  std::string iiwa_sdf_path;
  switch (collision_model) {
    case IiwaCollisionModel::kNoCollision:
      iiwa_sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/sdf/"
          "iiwa14_no_collision.urdf");
      break;
    case IiwaCollisionModel::kPolytopeCollision:
      iiwa_sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/sdf/"
          "iiwa14_polytope_collision.sdf");
      break;
    default:
      DRAKE_ABORT_MSG("Unrecognized collision_model.");
  }

  iiwa_model_ = parser.AddModelFromFile(iiwa_sdf_path, "iiwa");
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0", iiwa_model_));

  // Add the Schunk gripper and weld it to the end of the IIWA.
  const std::string wsg_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/"
      "wsg_50_description/sdf/schunk_wsg_50.sdf");
  wsg_model_ =
      parser.AddModelFromFile(wsg_sdf_path, "gripper");

  // Inspired by iiwa14_schunk_gripper but hand-tuned to get visible
  // alignment of sim + actual robot meshes in Director.
  // Underlying frames might be slightly different between the SDF this
  // sim uses + the URDFs Spartan uses...
  const RigidTransform<double> tool0_pose(RollPitchYaw<double>(0, -M_PI_2, 0),
                                        Vector3d(0, 0, 0.045));
  const RigidTransform<double> wsg_pose(RollPitchYaw<double>(M_PI, 0.3926, M_PI_2),
                                        Vector3d(0.04, 0, 0.0));

  plant_->WeldFrames(plant_->GetFrameByName("iiwa_link_7", iiwa_model_),
                     plant_->GetFrameByName("body", wsg_model_),
                     (tool0_pose*wsg_pose).GetAsIsometry3());

  plant_->template AddForceElement<multibody::UniformGravityFieldElement>(
      -9.81 * Vector3d::UnitZ());

  // Build the controller's version of the plant, which only contains the
  // IIWA and the equivalent inertia of the gripper.
  Parser controller_parser(owned_controller_plant_.get());
  const auto controller_iiwa_model =
      controller_parser.AddModelFromFile(iiwa_sdf_path, "iiwa");
  owned_controller_plant_->WeldFrames(owned_controller_plant_->world_frame(),
                                      owned_controller_plant_->GetFrameByName(
                                          "iiwa_link_0", controller_iiwa_model),
                                      Isometry3d::Identity());
  // Add a single body to represent the IIWA pendant's calibration of the
  // gripper.  The body of the WSG accounts for >90% of the total mass
  // (according to the sdf)... and we don't believe our inertia calibration
  // on the hardware to be so precise, so we simply ignore the inertia
  // contribution from the fingers here.
  const multibody::RigidBody<T>& wsg_equivalent =
      owned_controller_plant_->AddRigidBody(
          "wsg_equivalent", controller_iiwa_model,
          MakeCompositeGripperInertia(wsg_sdf_path));
  owned_controller_plant_->WeldFrames(owned_controller_plant_->GetFrameByName(
                                          "iiwa_link_7", controller_iiwa_model),
                                      wsg_equivalent.body_frame(),
                                      wsg_pose.GetAsIsometry3());

  owned_controller_plant_
      ->template AddForceElement<multibody::UniformGravityFieldElement>(
          -9.81 * Vector3d::UnitZ());

  // TODO(gizatt): Add manipulands
}

template <typename T>
void KukaSchunkStation<T>::Finalize() {
  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of my objects are added, and
  //   - cannot wire up my diagram until we have finalized the plant.

  plant_->Finalize();

  systems::DiagramBuilder<T> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  // Export the commanded positions via a PassThrough.
  auto iiwa_position =
      builder.template AddSystem<systems::PassThrough>(kNumDofIiwa);
  builder.ExportInput(iiwa_position->get_input_port(), "iiwa_position");
  builder.ExportOutput(iiwa_position->get_output_port(),
                       "iiwa_position_commanded");

  // Export iiwa "state" outputs.
  {
    auto demux = builder.template AddSystem<systems::Demultiplexer>(
        2 * kNumDofIiwa, kNumDofIiwa);
    builder.Connect(plant_->get_continuous_state_output_port(iiwa_model_),
                    demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0), "iiwa_position_measured");
    builder.ExportOutput(demux->get_output_port(1), "iiwa_velocity_estimated");

    builder.ExportOutput(plant_->get_continuous_state_output_port(iiwa_model_),
                         "iiwa_state_estimated");
  }

  builder.ExportInput(plant_->get_applied_spatial_force_input_port(), "applied_spatial_force");

  // Add the IIWA controller "stack".
  {
    owned_controller_plant_->Finalize();

    // Add the inverse dynamics controller.
    VectorXd iiwa_kp = VectorXd::Constant(kNumDofIiwa, 100);
    VectorXd iiwa_kd(kNumDofIiwa);
    for (int i = 0; i < kNumDofIiwa; i++) {
      // Critical damping gains.
      iiwa_kd[i] = 2 * std::sqrt(iiwa_kp[i]);
    }
    VectorXd iiwa_ki = VectorXd::Constant(kNumDofIiwa, 1);
    auto iiwa_controller = builder.template AddSystem<
        systems::controllers::InverseDynamicsController>(
        *owned_controller_plant_, iiwa_kp, iiwa_ki, iiwa_kd, false);
    iiwa_controller->set_name("iiwa_controller");
    builder.Connect(plant_->get_continuous_state_output_port(iiwa_model_),
                    iiwa_controller->get_input_port_estimated_state());

    // Add in feedforward torque.
    auto adder = builder.template AddSystem<systems::Adder>(2, kNumDofIiwa);
    builder.Connect(iiwa_controller->get_output_port_control(),
                    adder->get_input_port(0));
    builder.ExportInput(adder->get_input_port(1), "iiwa_feedforward_torque");
    builder.Connect(adder->get_output_port(),
                    plant_->get_actuation_input_port(iiwa_model_));

    // Approximate desired state command from a discrete derivative of the
    // position command input port.  This is implemented as a LinearSystem
    // with state variables to store the last position command.
    //    x[n+1] = u[n]
    //    y[n] = [u[n]; (u[n] - x[n])/h]
    // where u[n] is the positions, y[n] output the positions and
    // velocities, and h is the timestep.
    const double time_step = plant_->time_step();
    MatrixXd C(2 * kNumDofIiwa, kNumDofIiwa), D(2 * kNumDofIiwa, kNumDofIiwa);
    // clang-format off
    C << MatrixXd::Zero(kNumDofIiwa, kNumDofIiwa),
         -MatrixXd::Identity(kNumDofIiwa, kNumDofIiwa) / time_step;
    D << MatrixXd::Identity(kNumDofIiwa, kNumDofIiwa),
         MatrixXd::Identity(kNumDofIiwa, kNumDofIiwa) / time_step;
    // Approximate desired state command from a discrete derivative of the
    // position command input port.
    auto desired_state_from_position = builder.template AddSystem<
      systems::StateInterpolatorWithDiscreteDerivative>(kNumDofIiwa,
                                                        plant_->time_step());
    desired_state_from_position->set_name("desired_state_from_position");
    builder.Connect(desired_state_from_position->get_output_port(),
                    iiwa_controller->get_input_port_desired_state());
    builder.Connect(iiwa_position->get_output_port(),
                    desired_state_from_position->get_input_port());

    // Export commanded torques:
    builder.ExportOutput(adder->get_output_port(), "iiwa_torque_commanded");
    builder.ExportOutput(adder->get_output_port(), "iiwa_torque_measured");
  }

  {
    auto wsg_controller = builder.template AddSystem<
        manipulation::schunk_wsg::SchunkWsgPositionController>();
    wsg_controller->set_name("wsg_controller");

    builder.Connect(wsg_controller->get_generalized_force_output_port(),
                    plant_->get_actuation_input_port(wsg_model_));
    builder.Connect(plant_->get_continuous_state_output_port(wsg_model_),
                    wsg_controller->get_state_input_port());

    builder.ExportInput(wsg_controller->get_desired_position_input_port(),
                        "wsg_position");
    builder.ExportInput(wsg_controller->get_force_limit_input_port(),
                        "wsg_force_limit");

    auto wsg_mbp_state_to_wsg_state = builder.template AddSystem(
        manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<double>());
    builder.Connect(plant_->get_continuous_state_output_port(wsg_model_),
                    wsg_mbp_state_to_wsg_state->get_input_port());

    builder.ExportOutput(wsg_mbp_state_to_wsg_state->get_output_port(),
                         "wsg_state_measured");

    builder.ExportOutput(wsg_controller->get_grip_force_output_port(),
                         "wsg_force_measured");
  }

  builder.ExportOutput(
      plant_->get_generalized_contact_forces_output_port(iiwa_model_),
      "iiwa_torque_external");
  builder.ExportOutput(
      plant_->get_geometry_poses_output_port(),
      "geometry_poses");

  {  // Scene graph and RGB-D Cameras
    auto render_scene_graph =
        builder.template AddSystem<geometry::dev::SceneGraph>(*scene_graph_);

    builder.Connect(plant_->get_geometry_poses_output_port(),
                    render_scene_graph->get_source_pose_port(
                        plant_->get_source_id().value()));
  }

  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle");
  builder.ExportOutput(scene_graph_->get_query_output_port(),
                       "query");

  builder.ExportOutput(plant_->get_contact_results_output_port(),
      "contact_results");
  builder.ExportOutput(plant_->get_continuous_state_output_port(),
      "plant_continuous_state");

  builder.BuildInto(this);
}

template <typename T>
VectorX<T> KukaSchunkStation<T>::GetIiwaPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  // TODO(russt): update upon resolution of #9623.
  VectorX<T> q(kNumDofIiwa);
  for (int i = 0; i < kNumDofIiwa; i++) {
    q(i) = plant_
               ->template GetJointByName<RevoluteJoint>("iiwa_joint_" +
                                                        std::to_string(i + 1))
               .get_angle(plant_context);
  }
  return q;
}

template <typename T>
void KukaSchunkStation<T>::SetIiwaPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& q) const {
  const int num_iiwa_positions =
      plant_->num_positions(iiwa_model_);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q.size() == num_iiwa_positions);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetPositions(plant_context, &plant_state, iiwa_model_,
                       q);

  // Set the position history in the state interpolator to match.
  const auto& state_from_position = dynamic_cast<
      const systems::StateInterpolatorWithDiscreteDerivative<double>&>(
      this->GetSubsystemByName("desired_state_from_position"));
  state_from_position.set_initial_position(
      &this->GetMutableSubsystemState(state_from_position, state), q);
}

template <typename T>
VectorX<T> KukaSchunkStation<T>::GetIiwaVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  return plant_->GetVelocities(plant_context, iiwa_model_);
}

template <typename T>
void KukaSchunkStation<T>::SetIiwaVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& v) const {
  const int num_iiwa_velocities =
      plant_->num_velocities(iiwa_model_);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(v.size() == num_iiwa_velocities);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetVelocities(plant_context, &plant_state, iiwa_model_,
                        v);
}

template <typename T>
T KukaSchunkStation<T>::GetWsgPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> positions =
      plant_->GetPositions(plant_context, wsg_model_);
  return positions(1) - positions(0);
}

template <typename T>
T KukaSchunkStation<T>::GetWsgVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> velocities =
      plant_->GetVelocities(plant_context, wsg_model_);
  return velocities(1) - velocities(0);
}

template <typename T>
void KukaSchunkStation<T>::SetWsgPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& q) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  const Vector2<T> positions(-q / 2, q / 2);
  plant_->SetPositions(plant_context, &plant_state, wsg_model_,
                       positions);

  // Set the position history in the state interpolator to match.
  const auto& wsg_controller = dynamic_cast<
      const manipulation::schunk_wsg::SchunkWsgPositionController&>(
      this->GetSubsystemByName("wsg_controller"));
  wsg_controller.set_initial_position(
      &this->GetMutableSubsystemState(wsg_controller, state), q);
}

template <typename T>
void KukaSchunkStation<T>::SetWsgVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& v) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  const Vector2<T> velocities(-v / 2, v / 2);
  plant_->SetVelocities(plant_context, &plant_state, wsg_model_,
                        velocities);
}


}  // namespace drake_iiwa_sim

// TODO(russt): Support at least NONSYMBOLIC_SCALARS.  See #9573.
//   (and don't forget to include default_scalars.h)
template class ::drake_iiwa_sim::KukaSchunkStation<
    double>;
