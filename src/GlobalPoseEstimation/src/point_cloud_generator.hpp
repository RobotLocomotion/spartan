#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/eigen_types.h"

#include "yaml-cpp/yaml.h"

class PointCloudGenerator {
  public:
    PointCloudGenerator(const YAML::Node& config,
                        const std::string& dirname = "");

    Eigen::Matrix3Xd samplePointCloud();
    Eigen::Matrix3Xd samplePointCloudFromSurface();
    Eigen::Matrix3Xd samplePointCloudFromRaycast();

    RigidBodyTree<double> & get_robot() {
      return robot_;
    }

    Eigen::VectorXd get_q_robot() { 
      return q_robot_;
    }

  private:
    YAML::Node config_;
    RigidBodyTree<double> robot_;
    Eigen::VectorXd q_robot_;
};