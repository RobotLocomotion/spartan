#pragma once

/***
  Based heavily on
  `drake/examples/manipulation_station/manipulation_station.cc`.

  TODO(gizatt) Merge functionality with Drake as that system
  evolves to be more modular -- e.g., the Drake and WSG
  setup could be separated from that system and used here.
  (I'm not using it because, right now, the manipulation station
  class contains the environment + cameras of the 6.881 setup,
  which we don't share.)
***/

#include <memory>
#include "yaml-cpp/yaml.h"

#include "drake/common/eigen_types.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake_iiwa_sim {

/// Determines which sdf is loaded for the IIWA in the KukaSchunkStation.
enum class IiwaCollisionModel { kNoCollision, kPolytopeCollision };

using namespace drake;

template <typename T>
class KukaSchunkStation : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KukaSchunkStation)

  /// Construct the station model.
  ///
  /// @param time_step The time step used by MultibodyPlant<T>, and by the
  ///   discrete derivative used to approximate velocity from the position
  ///   command inputs.
  /// @param collision_model Determines which sdf is loaded for the IIWA.
  KukaSchunkStation(
      const YAML::Node& station_config,
      double time_step = 0.002,
      IiwaCollisionModel collision_model = IiwaCollisionModel::kNoCollision);

  // TODO(russt): Add scalar copy constructor etc once we support more
  // scalar types than T=double.  See #9573.

  /// Users *must* call Finalize() after making any additions to the
  /// multibody plant and before using this class in the Systems framework.
  /// This should be called exactly once.
  ///
  /// @see multibody::MultibodyPlant<T>::Finalize()
  void Finalize();

  /// Returns a reference to the main plant responsible for the dynamics of
  /// the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const multibody::MultibodyPlant<T>& get_multibody_plant()
  const {
    return *plant_;
  }

  /// Returns a mutable reference to the main plant responsible for the
  /// dynamics of the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  multibody::MultibodyPlant<T>& get_mutable_multibody_plant() {
    return *plant_;
  }

  /// Returns a reference to the SceneGraph responsible for all of the geometry
  /// for the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const geometry::SceneGraph<T>& get_scene_graph() const {
    return *scene_graph_;
  }

  /// Returns a mutable reference to the SceneGraph responsible for all of the
  /// geometry for the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  geometry::SceneGraph<T>& get_mutable_scene_graph() { return *scene_graph_; }

  /// Return a reference to the plant used by the inverse dynamics controller
  /// (which contains only a model of the iiwa + equivalent mass of the
  /// gripper).
  const multibody::MultibodyPlant<T>& get_controller_plant()
      const {
    return *owned_controller_plant_;
  }

  /// Get the number of joints in the IIWA (only -- does not include the
  /// gripper).
  int num_iiwa_joints() const { return 7; }

  /// Convenience method for getting all of the joint angles of the Kuka IIWA.
  /// This does not include the gripper.
  VectorX<T> GetIiwaPosition(const systems::Context<T>& station_context) const;

  /// Convenience method for setting all of the joint angles of the Kuka IIWA.
  /// Also sets the position history in the velocity command generator.
  /// @p q must have size num_iiwa_joints().
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetIiwaPosition(const systems::Context<T>& station_context,
                       systems::State<T>* state,
                       const Eigen::Ref<const VectorX<T>>& q) const;

  /// Convenience method for setting all of the joint angles of the Kuka IIWA.
  /// Also sets the position history in the velocity command generator.
  /// @p q must have size num_iiwa_joints().
  void SetIiwaPosition(systems::Context<T>* station_context,
                       const Eigen::Ref<const VectorX<T>>& q) const {
    SetIiwaPosition(*station_context, &station_context->get_mutable_state(), q);
  }

  /// Convenience method for getting all of the joint velocities of the Kuka
  // IIWA.  This does not include the gripper.
  VectorX<T> GetIiwaVelocity(const systems::Context<T>& station_context) const;

  /// Convenience method for setting all of the joint velocities of the Kuka
  /// IIWA. @v must have size num_iiwa_joints().
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetIiwaVelocity(const systems::Context<T>& station_context,
                       systems::State<T>* state,
                       const Eigen::Ref<const VectorX<T>>& v) const;

  /// Convenience method for setting all of the joint velocities of the Kuka
  /// IIWA. @v must have size num_iiwa_joints().
  void SetIiwaVelocity(systems::Context<T>* station_context,
                       const Eigen::Ref<const VectorX<T>>& v) const {
    SetIiwaVelocity(*station_context, &station_context->get_mutable_state(), v);
  }

  /// Convenience method for getting the position of the Schunk WSG. Note
  /// that the WSG position is the signed distance between the two fingers
  /// (not the state of the fingers individually).
  T GetWsgPosition(const systems::Context<T>& station_context) const;

  /// Convenience method for getting the velocity of the Schunk WSG.
  T GetWsgVelocity(const systems::Context<T>& station_context) const;

  /// Convenience method for setting the position of the Schunk WSG. Also
  /// sets the position history in the velocity interpolator.  Note that the
  /// WSG position is the signed distance between the two fingers (not the
  /// state of the fingers individually).
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetWsgPosition(const systems::Context<T>& station_context,
                      systems::State<T>* state, const T& q) const;

  /// Convenience method for setting the position of the Schunk WSG. Also
  /// sets the position history in the velocity interpolator.  Note that the
  /// WSG position is the signed distance between the two fingers (not the
  /// state of the fingers individually).
  void SetWsgPosition(systems::Context<T>* station_context, const T& q) const {
    SetWsgPosition(*station_context, &station_context->get_mutable_state(), q);
  }

  /// Convenience method for setting the velocity of the Schunk WSG.
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetWsgVelocity(const systems::Context<T>& station_context,
                      systems::State<T>* state, const T& v) const;

  /// Convenience method for setting the velocity of the Schunk WSG.
  void SetWsgVelocity(systems::Context<T>* station_context, const T& v) const {
    SetWsgVelocity(*station_context, &station_context->get_mutable_state(), v);
  }


 private:
  // These are only valid until Finalize() is called.
  std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<geometry::SceneGraph<T>> owned_scene_graph_;

  // These are valid for the lifetime of this system.
  std::unique_ptr<multibody::MultibodyPlant<T>>
      owned_controller_plant_;
  multibody::MultibodyPlant<T>* plant_;
  geometry::SceneGraph<T>* scene_graph_;

  multibody::ModelInstanceIndex iiwa_model_;
  multibody::ModelInstanceIndex wsg_model_;

  YAML::Node station_config_;
};

}  // namespace drake_iiwa_sim
