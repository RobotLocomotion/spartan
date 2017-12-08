#pragma once

#include <unistd.h>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "drake/multibody/rigid_body_tree.h"

#include "yaml-cpp/yaml.h"

Eigen::VectorXd compute_articulated_icp_update_for_points(
    RigidBodyTree<double>& robot, const Eigen::VectorXd& q_robot,
    const Eigen::Matrix3Xd& points, double prior_weight,
    double outlier_rejection_portion, bool use_point_to_plane);