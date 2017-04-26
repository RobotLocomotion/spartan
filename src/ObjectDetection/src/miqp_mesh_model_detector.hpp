#pragma once

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/gurobi_solver.h"

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
      double lower_bound;
    };

    struct TransformationVars {
      drake::solvers::VectorDecisionVariable<3> T;
      drake::solvers::MatrixDecisionVariable<3,3> R;
      std::tuple<std::vector<drake::solvers::MatrixDecisionVariable<3, 3>>,
           std::vector<drake::solvers::MatrixDecisionVariable<3, 3>>,
           std::vector<drake::solvers::MatrixDecisionVariable<3, 3>>,
           std::vector<drake::solvers::MatrixDecisionVariable<3, 3>>> R_indicators;
    };

    MIQPMultipleMeshModelDetector(YAML::Node config);
  
    void handleMipSolCallbackFunction(const drake::solvers::MathematicalProgram& prog, const drake::solvers::GurobiSolver::SolveStatusInfo& solve_info);
    void handleMipNodeCallbackFunction(const drake::solvers::MathematicalProgram& prog, const drake::solvers::GurobiSolver::SolveStatusInfo& solve_info,
      Eigen::VectorXd& vals, drake::solvers::VectorXDecisionVariable& vars);

    void doScenePointPreprocessing(const Eigen::Matrix3Xd& scene_pts_in, Eigen::Matrix3Xd& scene_pts_out);
    void doModelPointSampling(Eigen::Matrix3Xd& pts, Eigen::MatrixXd& B);

    void collectBodyMeshesFromRBT(Eigen::Matrix3Xd& all_vertices, 
                                  DrakeShapes::TrianglesVector& all_faces, 
                                  std::vector<int>& face_body_map);


    std::vector<TransformationVars> addTransformationVarsAndConstraints(
                            drake::solvers::MathematicalProgram& prog,
                            bool world_to_body_direction);

    void getInitialGuessFromRobotState(const Eigen::VectorXd& q_robot, 
      Eigen::VectorXd& vals, drake::solvers::VectorXDecisionVariable& vars);

    std::vector<Solution> doObjectDetectionWithWorldToBodyFormulation(const Eigen::Matrix3Xd& scene_pts);
    std::vector<Solution> doObjectDetectionWithWorldToBodyFormulationSampledModelPoints(const Eigen::Matrix3Xd& scene_pts);
    std::vector<Solution> doObjectDetectionWithBodyToWorldFormulation(const Eigen::Matrix3Xd& scene_pts);
    std::vector<Solution> doObjectDetection(const Eigen::Matrix3Xd& scene_pts);
    
    void doICPProcessing();

    RigidBodyTree<double> & get_robot() {
      return robot_;
    }

    struct SolveHistoryElem {
      double wall_time;
      double reported_runtime;
      double best_objective;
      double best_bound;
      int explored_node_count;
      int feasible_solutions_count;
    };
    const std::vector<SolveHistoryElem>& get_solve_history() {
      return solve_history;
    }

  private:
    RigidBodyTree<double> robot_;
    Eigen::VectorXd q_robot_gt_;

    YAML::Node config_;

    // Big-M used
    double optBigNumber_ = 100;

    // POINT CLOUD PREPROCESSING OPTIONS
    // Point cloud downsampling amount, -1 is no downsampling
    int optDownsampleToThisManyPoints_ = -1;
    // Inject outliers into point cloud (by replacing existing points?)
    int optNumOutliers_ = 0;
    // Injected scene noise
    double optAddedSceneNoise_ = 0.0;
    int optScenePointRandSeed_ = -1;
    // If we sample the model, use this many rays
    int optModelSampleRays_ = 10;
    int optModelPointRandSeed_ = -1;

    
    // SOLVER OPTIONS
    
    // Rotation constraint options
    int optRotationConstraint_ = 4;
    int optRotationConstraintNumFaces_ = 2;

    // Outlier options
    bool optAllowOutliers_ = true;
    double optPhiMax_ = 0.1;

    // Use initial guess, and if so, corruption amount
    bool optUseInitialGuess_ = false;
    double optCorruption_ = 100.0;
    int optInitGuessRandSeed_ = -1;

    // ICP heuristic and internal parameters
    bool optUseICPHeuristic_ = false;
    double optICPPriorWeight_ = 1.0;
    int optICPIters_ = 1000;
    double optICPRejectionProp_ = 0.0;

    std::vector<MIQPMultipleMeshModelDetector::TransformationVars> transform_by_object_;
    drake::solvers::MatrixXDecisionVariable phi_;
    drake::solvers::MatrixXDecisionVariable alpha_;
    drake::solvers::MatrixXDecisionVariable C_;
    drake::solvers::MatrixXDecisionVariable f_;
    drake::solvers::MatrixXDecisionVariable f_outlier_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd B_;


    Eigen::Matrix3Xd all_vertices_;
    DrakeShapes::TrianglesVector all_faces_;
    std::vector<int> face_body_map_; // Maps from a face index (into all_faces) to an RBT body index.
    Eigen::Matrix3Xd scene_pts_;

    double best_heuristic_supplied_yet_ = std::numeric_limits<double>::infinity();
    double last_published_node_ = 0.0;
    double last_published_sol_ = 0.0;
    double best_sol_objective_yet_ = std::numeric_limits<double>::infinity();
    std::vector<SolveHistoryElem> solve_history; 

    // First in, last out structure to let us queue
    // up seeds for ICP-based solution improvement.
    // (We want a stack and not a queue so that we
    //  grab the most recent, and hopefully "best", solution
    //  from the solver to seed our search.)
    std::mutex icp_search_seeds_lock_;
    std::stack<Eigen::VectorXd> icp_search_seeds_;
    struct NewHeuristicSol {
      Eigen::VectorXd vals;
      drake::solvers::VectorXDecisionVariable vars;
    };
    std::mutex new_heuristic_sols_lock_;
    std::deque<NewHeuristicSol> new_heuristic_sols_;

};