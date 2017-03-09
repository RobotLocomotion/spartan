#pragma once

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_tree.h"

class RemoteTreeViewerWrapper {
  public:
    RemoteTreeViewerWrapper();
    void publishPointCloud(const Eigen::Matrix3Xd pts, std::vector<std::string> path);
    void publishLine(const Eigen::Matrix3Xd pts, std::vector<std::string> path);
    void publishRawMesh(const Eigen::Matrix3Xd verts, std::vector<Eigen::Vector3i> tris, std::vector<std::string> path);
    void publishRigidBodyTree(const RigidBodyTree<double>& tree, const Eigen::VectorXd q, const Eigen::Vector4d color, std::vector<std::string> path, bool visual = true);
    void publishRigidBody(const RigidBody<double>& body, Eigen::Affine3d tf, const Eigen::Vector4d color, std::vector<std::string> path);
    void publishGeometry(const DrakeShapes::Geometry& geometry, Eigen::Affine3d tf, const Eigen::Vector4d color, std::vector<std::string> path);

  private:
    drake::lcm::DrakeLcm lcm_;
};