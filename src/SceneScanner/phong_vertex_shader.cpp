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
using drake::systems::rendering::PoseVector;
using namespace drake::systems;

template <typename T>
PhongVertexShaderInput<T>::PhongVertexShaderInput(int n_vertices)
    : n_vertices_(n_vertices), BasicVector<double>(n_vertices * 6) {
  this->SetFromVector(VectorX<double>::Zero(n_vertices * 6));
}
template <typename T>
PhongVertexShaderInput<T>* PhongVertexShaderInput<T>::DoClone() const {
  return new PhongVertexShaderInput(n_vertices_);
}
template class PhongVertexShaderInput<double>;

template <typename T>
PhongVertexShaderOutput<T>::PhongVertexShaderOutput(int n_vertices)
    : n_vertices_(n_vertices), BasicVector<double>(n_vertices * 3) {
  this->SetFromVector(VectorX<double>::Zero(n_vertices * 3));
}
template <typename T>
PhongVertexShaderOutput<T>* PhongVertexShaderOutput<T>::DoClone() const {
  return new PhongVertexShaderOutput(n_vertices_);
}
template class PhongVertexShaderOutput<double>;

PhongVertexShader::PhongVertexShader(const std::string& name,
                                         int n_vertices, int n_lights)
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
}

void PhongVertexShader::CalcVertexOutput(
    const Context<double>& context,
    PhongVertexShaderOutput<double>* data_output) const {
  printf("NOPE!\n");
  return;
}

std::ostream& operator<<(std::ostream& out, const PhongVertexShader& sensor) {
  out << "Self-printing not implemented yet for PhongVertexShader system.";

  return out;
}