#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/rendering/pose_vector.h"

/// Specializes BasicVector for the Phong Fragment shader N-elementRGB output.
template <typename T>
class PhongFragmentShaderOutput : public drake::systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongFragmentShaderOutput)

  /// Default constructor.  Sets all rows to zero.
  ///
  /// @param[in] n_vertices The number of vertices the system consumes in a
  /// batch.
  explicit PhongFragmentShaderOutput(int n_vertices);

 protected:
  PhongFragmentShaderOutput* DoClone() const override;

 private:
  const int n_vertices_{};
};

/// Specializes BasicVector for the Phong Fragment shader N-element
/// oriented vertex input.
template <typename T>
class PhongFragmentShaderInput : public drake::systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongFragmentShaderInput)

  /// Default constructor.  Sets all rows to zero.
  ///
  /// @param[in] n_vertices The number of vertices the system consumes in a
  /// batch.
  explicit PhongFragmentShaderInput(int n_vertices);

 protected:
  PhongFragmentShaderInput* DoClone() const override;

 private:
  const int n_vertices_{};
};

/// Input:
///   - A 6DOF camera pose
///   - A list of N vertex/normal pairs (N fixed at creation)
///
/// Output:
///    - N RGB values produced by corresponding vertices
///      according to a phong model.
///
/// Parameters:
///    - A discrete number of lights, fixed at creation of this system,
///      each with their own diffuse and specular colors + position.
///      TODO(gizatt): # of lights is a parameter too! But that's a lot
///      harder...
///    - Ambient, diffuse, and specular colors for each of the N vertices.

class PhongFragmentShader : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongFragmentShader)

  /// A %PhongFragmentShader constructor.
  ///
  /// @param[in] name A name for the system.
  ///
  /// @param[in] n_vertices The number of input verts the shader will accept
  /// in a batch.
  ///
  /// @param[in] n_lights The number of lights the shader will support.
  PhongFragmentShader(const std::string& name, int n_vertices, int n_lights);

  /// Returns a descriptor of the camera pose input port.
  const drake::systems::InputPortDescriptor<double>& get_camera_pose_input_port() const;

  /// Returns a descriptor of the vertex/normal pairs input port.
  const drake::systems::InputPortDescriptor<double>& get_fragment_input_port() const;

  /// Returns the RGB value output port.
  const drake::systems::OutputPort<double>& get_fragment_output_port() const;

  friend std::ostream& operator<<(
      std::ostream& out, const PhongFragmentShader& phong_fragment_shader);

 private:
  // These are calculators for the output.
  void CalcFragmentOutput(const drake::systems::Context<double>& context,
                          PhongFragmentShaderOutput<double>* data_output) const;

  const std::string name_;

  const int n_vertices_{};
  const int n_lights_{};

  int camera_pose_input_port_index_{};
  int fragment_input_port_index_{};
  int fragment_output_port_index_{};

  // int input_port_index_{};
  // int depth_output_port_index_{};
  // int pose_output_port_index_{};
};