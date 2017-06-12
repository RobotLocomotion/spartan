#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/eigen_types.h"
#include "yaml-cpp/yaml.h"
#include <boost/python.hpp>

std::vector<double> scaleTime(RigidBodyTree<double>* model, std::vector<Eigen::VectorXd> q_sol, const char* constraintFileString);

