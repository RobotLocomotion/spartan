#include "phong_fragment_shader.h"

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
PhongFragmentShaderInput<T>::PhongFragmentShaderInput(int n_vertices)
    : n_vertices_(n_vertices), BasicVector<double>(n_vertices * 6) {
  this->SetFromVector(VectorX<double>::Zero(n_vertices * 6));
}
template <typename T>
PhongFragmentShaderInput<T>* PhongFragmentShaderInput<T>::DoClone() const {
  return new PhongFragmentShaderInput(n_vertices_);
}
template class PhongFragmentShaderInput<double>;

template <typename T>
PhongFragmentShaderOutput<T>::PhongFragmentShaderOutput(int n_vertices)
    : n_vertices_(n_vertices), BasicVector<double>(n_vertices * 3) {
  this->SetFromVector(VectorX<double>::Zero(n_vertices * 3));
}
template <typename T>
PhongFragmentShaderOutput<T>* PhongFragmentShaderOutput<T>::DoClone() const {
  return new PhongFragmentShaderOutput(n_vertices_);
}
template class PhongFragmentShaderOutput<double>;

PhongFragmentShader::PhongFragmentShader(const std::string& name,
                                         int n_vertices, int n_lights)
    : name_(name), n_vertices_(n_vertices), n_lights_(n_lights) {
  camera_pose_input_port_index_ =
      DeclareVectorInputPort(PoseVector<double>()).get_index();

  fragment_input_port_index_ =
      DeclareVectorInputPort(PhongFragmentShaderInput<double>(n_vertices))
          .get_index();

  fragment_output_port_index_ =
      DeclareVectorOutputPort(PhongFragmentShaderOutput<double>(n_vertices),
                              &PhongFragmentShader::CalcFragmentOutput)
          .get_index();
}

void PhongFragmentShader::CalcFragmentOutput(
    const Context<double>& context,
    PhongFragmentShaderOutput<double>* data_output) const {
  printf("NOPE!\n");
  return;
}

std::ostream& operator<<(std::ostream& out, const PhongFragmentShader& sensor) {
  out << "Self-printing not implemented yet for PhongFragmentShader system.";

  return out;
}