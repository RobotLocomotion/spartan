#include "phong_vertex_shader.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/rendering/pose_vector.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix;

using std::make_unique;
using std::move;

using drake::VectorX;
using drake::Vector3;
using drake::systems::rendering::PoseVector;
using namespace drake::systems;

template <typename T>
PhongVertexShaderInput<T>::PhongVertexShaderInput(int n_vertices)
    : n_vertices_(n_vertices), BasicVector<double>(n_vertices * 6) {
  this->SetFromVector(VectorX<T>::Zero(n_vertices * 6));
}
template <typename T>
PhongVertexShaderInput<T>* PhongVertexShaderInput<T>::DoClone() const {
  return new PhongVertexShaderInput(n_vertices_);
}
template <typename T>
void PhongVertexShaderInput<T>::SetVertex(
    int index, const Eigen::Ref<const Vector3<T>>& v) {
  (*this)[index * 6 + 0] = v[0];
  (*this)[index * 6 + 1] = v[1];
  (*this)[index * 6 + 2] = v[2];
}
template <typename T>
void PhongVertexShaderInput<T>::SetNormal(
    int index, const Eigen::Ref<const Vector3<T>>& n) {
  (*this)[index * 6 + 3] = n[0];
  (*this)[index * 6 + 4] = n[1];
  (*this)[index * 6 + 5] = n[2];
}
template <typename T>
Vector3<T> PhongVertexShaderInput<T>::GetVertex(int index) const {
  return Vector3<T>((*this)[index * 6 + 0], (*this)[index * 6 + 1],
                    (*this)[index * 6 + 2]);
}
template <typename T>
Vector3<T> PhongVertexShaderInput<T>::GetNormal(int index) const {
  return Vector3<T>((*this)[index * 6 + 3], (*this)[index * 6 + 4],
                    (*this)[index * 6 + 5]);
}
template class PhongVertexShaderInput<double>;

//////////////////////////////////////////
template <typename T>
PhongVertexShaderOutput<T>::PhongVertexShaderOutput(int n_vertices)
    : n_vertices_(n_vertices), BasicVector<double>(n_vertices * 3) {
  this->SetFromVector(VectorX<T>::Zero(n_vertices * 3));
}
template <typename T>
PhongVertexShaderOutput<T>* PhongVertexShaderOutput<T>::DoClone() const {
  return new PhongVertexShaderOutput(n_vertices_);
}
template <typename T>
void PhongVertexShaderOutput<T>::SetRGB(
    int index, const Eigen::Ref<const Vector3<T>>& rgb) {
  (*this)[index * 3 + 0] = rgb[0];
  (*this)[index * 3 + 1] = rgb[1];
  (*this)[index * 3 + 2] = rgb[2];
}
template <typename T>
Vector3<T> PhongVertexShaderOutput<T>::GetRGB(int index) const {
  return Vector3<T>((*this)[index * 3 + 0], (*this)[index * 3 + 1],
                    (*this)[index * 3 + 2]);
}
template class PhongVertexShaderOutput<double>;

//////////////////////////////////////////
template <typename T>
PhongVertexShaderMaterialParameters<T>::PhongVertexShaderMaterialParameters(
    int n_vertices)
    : n_vertices_(n_vertices), BasicVector<double>(n_vertices * 9) {
  this->SetFromVector(VectorX<T>::Zero(n_vertices * 9));
}
template <typename T>
PhongVertexShaderMaterialParameters<T>*
PhongVertexShaderMaterialParameters<T>::DoClone() const {
  return new PhongVertexShaderMaterialParameters(n_vertices_);
}
template <typename T>
void PhongVertexShaderMaterialParameters<T>::SetMaterial(
    int index, const PhongVertexShaderMaterialParameters::Material& material) {
  for (int i = 0; i < 3; i++) {
    (*this)[index * 9 + 0 + i] = material.ambient[i];
    (*this)[index * 9 + 3 + i] = material.diffuse[i];
    (*this)[index * 9 + 6 + i] = material.specular[i];
  }
}
template <typename T>
typename PhongVertexShaderMaterialParameters<T>::Material
PhongVertexShaderMaterialParameters<T>::GetMaterial(int index) const {
  Material material;
  for (int i = 0; i < 3; i++) {
    material.ambient[i] = (*this)[index * 9 + 0 + i];
    material.diffuse[i] = (*this)[index * 9 + 3 + i];
    material.specular[i] = (*this)[index * 9 + 6 + i];
  }
  return material;
}
template class PhongVertexShaderMaterialParameters<double>;

//////////////////////////////////////////
template <typename T>
PhongVertexShaderLightParameters<T>::PhongVertexShaderLightParameters(
    int n_lights)
    : n_lights_(n_lights), BasicVector<double>(n_lights * 12) {
  this->SetFromVector(VectorX<T>::Zero(n_lights * 12));
}
template <typename T>
PhongVertexShaderLightParameters<T>*
PhongVertexShaderLightParameters<T>::DoClone() const {
  return new PhongVertexShaderLightParameters(n_lights_);
}
template <typename T>
void PhongVertexShaderLightParameters<T>::SetLight(
    int index, const PhongVertexShaderLightParameters::Light& light) {
  for (int i = 0; i < 3; i++) {
    (*this)[index * 12 + 0 + i] = light.position[i];
    (*this)[index * 12 + 3 + i] = light.ambient[i];
    (*this)[index * 12 + 6 + i] = light.diffuse[i];
    (*this)[index * 12 + 9 + i] = light.specular[i];
  }
}
template <typename T>
typename PhongVertexShaderLightParameters<T>::Light
PhongVertexShaderLightParameters<T>::GetLight(int index) const {
  Light light;
  for (int i = 0; i < 3; i++) {
    light.position[i] = (*this)[index * 12 + 0 + i];
    light.ambient[i] = (*this)[index * 12 + 3 + i];
    light.diffuse[i] = (*this)[index * 12 + 6 + i];
    light.specular[i] = (*this)[index * 12 + 9 + i];
  }
  return light;
}
template class PhongVertexShaderLightParameters<double>;

//////////////////////////////////////////
//////////////////////////////////////////
//////////////////////////////////////////
PhongVertexShader::PhongVertexShader(const std::string& name, int n_vertices,
                                     int n_lights)
    : name_(name), n_vertices_(n_vertices), n_lights_(n_lights) {
  camera_pose_input_port_index_ =
      DeclareVectorInputPort(PoseVector<double>()).get_index();

  vertex_input_port_index_ =
      DeclareVectorInputPort(PhongVertexShaderInput<double>(n_vertices))
          .get_index();

  rgb_output_port_index_ =
      DeclareVectorOutputPort(PhongVertexShaderOutput<double>(n_vertices),
                              &PhongVertexShader::CalcVertexOutput)
          .get_index();

  material_parameters_index_ = this->DeclareNumericParameter(
      PhongVertexShaderMaterialParameters<double>(n_vertices));
  light_parameters_index_ = this->DeclareNumericParameter(
      PhongVertexShaderLightParameters<double>(n_lights));
}

void PhongVertexShader::CalcVertexOutput(
    const Context<double>& context,
    PhongVertexShaderOutput<double>* data_output) const {
  auto vertex_input = this->EvalVectorInput<PhongVertexShaderInput>(
      context, vertex_input_port_index_);
  auto camera_pose_input =
      this->EvalVectorInput<PoseVector>(context, camera_pose_input_port_index_);

  auto& material_parameters =
      this->GetNumericParameter<PhongVertexShaderMaterialParameters>(
          context, material_parameters_index_);
  auto& light_parameters =
      this->GetNumericParameter<PhongVertexShaderLightParameters>(
          context, light_parameters_index_);

  drake::Vector3<double> camera_T =
      camera_pose_input->get_translation().vector();

  // For every vertiex in the input...
  for (int i = 0; i < vertex_input->get_num_vertices(); i++) {
    // First pass: just use ambient lighting
    Vector3<double> rgb_intensity;
    rgb_intensity.setZero();
    for (int light_i = 0; light_i < light_parameters.get_num_lights();
         light_i++) {
      rgb_intensity += (material_parameters.GetMaterial(i).ambient.array() *
                        light_parameters.GetLight(light_i).ambient.array())
                           .matrix();
    }
    data_output->SetRGB(i, rgb_intensity);
  }

  return;
}

std::ostream& operator<<(std::ostream& out, const PhongVertexShader& sensor) {
  out << "Self-printing not implemented yet for PhongVertexShader system.";

  return out;
}