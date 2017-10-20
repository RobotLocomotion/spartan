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

/// Specializes BasicVector for the Phong Vertex shader N-element
/// oriented vertex input.
template <typename T>
class PhongVertexShaderInput : public drake::systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongVertexShaderInput)

  /// Default constructor.  Sets all rows to zero.
  ///
  /// @param[in] n_vertices The number of vertices the system consumes in a
  /// batch.
  explicit PhongVertexShaderInput(int n_vertices);

  int get_num_vertices() const { return n_vertices_; };

  void SetVertex(int index, const Eigen::Ref<const drake::Vector3<T>>& v);
  void SetNormal(int index, const Eigen::Ref<const drake::Vector3<T>>& n);
  drake::Vector3<T> GetVertex(int index) const;
  drake::Vector3<T> GetNormal(int index) const;

 protected:
  PhongVertexShaderInput* DoClone() const override;

 private:
  const int n_vertices_{};
};

/// Specializes BasicVector for the Phong vertex shader output.
/// Represents a list of RGB intensity values produced by each
/// corresponding input vertex.
template <typename T>
class PhongVertexShaderOutput : public drake::systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongVertexShaderOutput)

  /// Default constructor.  Sets all rows to zero.
  ///
  /// @param[in] n_vertices The number of vertices the system consumes in a
  /// batch.
  explicit PhongVertexShaderOutput(int n_vertices);

  int get_num_vertices() const { return n_vertices_; };

  void SetRGB(int index, const Eigen::Ref<const drake::Vector3<T>>& rgb);
  drake::Vector3<T> GetRGB(int index) const;

 protected:
  PhongVertexShaderOutput* DoClone() const override;

 private:
  const int n_vertices_{};
};

/// Specializes BasicVector for the Phong Vertex shader material parameters:
/// a vector of N material property sets for the N vertices.
/// Encodes ambient, diffuse, and specular materials for each vertex.
template <typename T>
class PhongVertexShaderMaterialParameters
    : public drake::systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongVertexShaderMaterialParameters)

  /// Default constructor.  Sets all rows to zero.
  ///
  /// @param[in] n_vertices The number of vertices the system consumes in a
  /// batch.
  explicit PhongVertexShaderMaterialParameters(int n_vertices);

  int get_num_vertices() const { return n_vertices_; };

  struct Material {
    drake::Vector3<T> ambient;
    drake::Vector3<T> diffuse;
    drake::Vector3<T> specular;
  };

  void SetMaterial(int index, const Material& material);
  Material GetMaterial(int index) const;

 protected:
  PhongVertexShaderMaterialParameters* DoClone() const override;

 private:
  const int n_vertices_{};
};

/// Specializes BasicVector for the Phong Vertex shader light properties
/// a vector of N light details for N lights.
/// Encodes ambient, diffuse, and specular sources, plus the light position,
/// for each light.
template <typename T>
class PhongVertexShaderLightParameters : public drake::systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongVertexShaderLightParameters)

  /// Default constructor.  Sets all rows to zero.
  ///
  /// @param[in] n_lights The number of lights in the system.
  explicit PhongVertexShaderLightParameters(int n_vertices);

  int get_num_lights() const { return n_lights_; };

  struct Light {
    drake::Vector3<T> position;
    drake::Vector3<T> ambient;
    drake::Vector3<T> diffuse;
    drake::Vector3<T> specular;
  };

  void SetLight(int index, const Light& material);
  Light GetLight(int index) const;

 protected:
  PhongVertexShaderLightParameters* DoClone() const override;

 private:
  const int n_lights_{};
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

class PhongVertexShader : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhongVertexShader)

  /// A %PhongVertexShader constructor.
  ///
  /// @param[in] name A name for the system.
  ///
  /// @param[in] n_vertices The number of input verts the shader will accept
  /// in a batch.
  ///
  /// @param[in] n_lights The number of lights the shader will support.
  PhongVertexShader(const std::string& name, int n_vertices, int n_lights);

  /// Returns a descriptor of the camera pose input port.
  const drake::systems::InputPortDescriptor<double>&
  get_camera_pose_input_port() const {
    return System<double>::get_input_port(camera_pose_input_port_index_);
  }

  /// Returns a descriptor of the vertex/normal pairs input port.
  const drake::systems::InputPortDescriptor<double>& get_vertex_input_port()
      const {
    return drake::systems::System<double>::get_input_port(
        vertex_input_port_index_);
  }

  PhongVertexShaderMaterialParameters<double>* get_material_parameters(
      drake::systems::Context<double>* context) {
    return drake::systems::LeafSystem<double>::GetMutableNumericParameter<
        PhongVertexShaderMaterialParameters>(context,
                                             material_parameters_index_);
  }

  PhongVertexShaderLightParameters<double>* get_light_parameters(
      drake::systems::Context<double>* context) {
    return drake::systems::LeafSystem<double>::GetMutableNumericParameter<
        PhongVertexShaderLightParameters>(context, light_parameters_index_);
  }

  /// Returns the RGB value output port.
  const drake::systems::OutputPort<double>& get_rgb_output_port() const {
    return drake::systems::System<double>::get_output_port(
        rgb_output_port_index_);
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const PhongVertexShader& phong_vertex_shader);

 private:
  // These are calculators for the output.
  void CalcVertexOutput(const drake::systems::Context<double>& context,
                        PhongVertexShaderOutput<double>* data_output) const;

  const std::string name_;

  const int n_vertices_{};
  const int n_lights_{};

  int camera_pose_input_port_index_{};
  int vertex_input_port_index_{};
  int rgb_output_port_index_{};

  int material_parameters_index_{};
  int light_parameters_index_{};

  // int input_port_index_{};
  // int depth_output_port_index_{};
  // int pose_output_port_index_{};
};