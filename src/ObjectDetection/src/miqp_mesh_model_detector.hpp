#pragma once

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

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
      double solve_time;
    };

    struct TransformationVars {
      drake::solvers::VectorDecisionVariable<3> T;
      drake::solvers::MatrixDecisionVariable<3,3> R;
    };

    MIQPMultipleMeshModelDetector(YAML::Node config);
  
    void doScenePointPreprocessing(const Eigen::Matrix3Xd& scene_pts_in, Eigen::Matrix3Xd& scene_pts_out);
    Eigen::Matrix3Xd doModelPointSampling();

    void collectBodyMeshesFromRBT(Eigen::Matrix3Xd& all_vertices, 
                                  DrakeShapes::TrianglesVector& all_faces, 
                                  std::vector<int>& face_body_map);


    std::vector<TransformationVars> addTransformationVarsAndConstraints(drake::solvers::MathematicalProgram& prog);

    std::vector<Solution> doObjectDetectionWithWorldToBodyFormulation(const Eigen::Matrix3Xd& scene_pts);
    std::vector<Solution> doObjectDetectionWithBodyToWorldFormulation(const Eigen::Matrix3Xd& scene_pts);
    std::vector<Solution> doObjectDetection(const Eigen::Matrix3Xd& scene_pts);

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
    int optModelSampleRays_ = 10;

};