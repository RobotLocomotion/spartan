#pragma once

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "drake/multibody/rigid_body_tree.h"

#include "yaml-cpp/yaml.h"

class MIQPMultipleMeshModelDetector {
  public:
    struct PointCorrespondence {
      Eigen::Vector3d scene_pt;
      Eigen::Vector3d model_pt;
      int face_ind;
      int scene_ind;
      std::vector<Eigen::Vector3d> model_verts;
      std::vector<double> vert_weights;
      std::vector<int> vert_inds;
    };

    struct ObjectDetection {
      Eigen::Affine3d est_tf;
      std::vector<PointCorrespondence> correspondences;
      int obj_ind;
    };

    struct Solution {
      std::vector<ObjectDetection> detections;
      double objective;
    };

    MIQPMultipleMeshModelDetector(YAML::Node config);

    std::vector<Solution> doObjectDetection(Eigen::Matrix3Xd scene_pts);

    RigidBodyTree<double> & get_robot() {
      return robot_;
    }

  private:
    RigidBodyTree<double> robot_;
    Eigen::VectorXd q_robot_gt_;

    YAML::Node config_;

    int optNumRays_ = 10;
    int optRotationConstraint_ = 4;
    int optRotationConstraintNumFaces_ = 2;
    int optDownsampleToThisManyPoints_ = -1;
    bool optAllowOutliers_ = true;
    double optPhiMax_ = 0.1;
    bool optUseInitialGuess_ = false;
    double optCorruption_ = 100.0;

};