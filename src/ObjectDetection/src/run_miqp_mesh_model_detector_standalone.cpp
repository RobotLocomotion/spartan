/*
 */

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/lcm/drake_lcm.h"

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"
#include "optimization_helpers.h"
#include "rotation_helpers.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>  

#include "RemoteTreeViewerWrapper.hpp"

#include "point_cloud_generator.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;
using namespace drake::systems;
using namespace drake::symbolic;

const double kBigNumber = 100;

class MILPMultipleMeshModelDetector {
  public:
    struct PointCorrespondence {
      Vector3d scene_pt;
      Vector3d model_pt;
      int face_ind;
      int scene_ind;
      std::vector<Vector3d> model_verts;
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

    MILPMultipleMeshModelDetector(YAML::Node config){
      if (config["rotation_constraint"])
        optRotationConstraint_ = config["rotation_constraint"].as<int>();
      if (config["rotation_constraint_num_faces"])
        optRotationConstraintNumFaces_ = config["rotation_constraint_num_faces"].as<int>();
      if (config["allow_outliers"])
        optAllowOutliers_ = config["allow_outliers"].as<bool>();
      if (config["phi_max"])
        optPhiMax_ = config["phi_max"].as<double>();
      if (config["use_initial_guess"])
        optUseInitialGuess_ = config["use_initial_guess"].as<bool>();
      if (config["corruption_amount"])
        optCorruption_ = config["corruption_amount"].as<double>();

      config_ = config;

      // Load the model itself
      if (config["models"] == NULL){
        runtime_error("Must specify models for object detector to work with.");
      }
      // Model will be a RigidBodyTree.
      q_robot_gt_.resize(0);
      int old_q_robot_gt_size = 0;
      for (auto iter=config["models"].begin(); iter!=config["models"].end(); iter++){
        string urdf = (*iter)["urdf"].as<string>();
        AddModelInstanceFromUrdfFileWithRpyJointToWorld(urdf, &robot_);
        // And add initial state info that we were passed
        vector<double> q0 = (*iter)["q0"].as<vector<double>>();
        assert(robot_.get_num_positions() - old_q_robot_gt_size == q0.size());
        q_robot_gt_.conservativeResize(robot_.get_num_positions());
        for (int i=0; i<q0.size(); i++){
          q_robot_gt_[old_q_robot_gt_size] = q0[i];
          old_q_robot_gt_size++; 
        }
      }
    }

    std::vector<Solution> doObjectDetection(Eigen::Matrix3Xd scene_pts){
      KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_gt_);

      // Extract vertices and meshes from the RBT.
      Matrix3Xd all_vertices;
      DrakeShapes::TrianglesVector all_faces;
      vector<int> face_body_map; // maps each face to body index
      
      // Collect faces from each body
      for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++) {
        const RigidBody<double>& body = robot_.get_body(body_i);
        auto collision_elems = body.get_collision_element_ids();
        for (const auto& collision_elem : body.get_collision_element_ids()) {
          auto element = robot_.FindCollisionElement(collision_elem);
          if (element->hasGeometry()){
            const DrakeShapes::Geometry & geometry = element->getGeometry();
            if (geometry.hasFaces()){
              Matrix3Xd points;
              geometry.getPoints(points);
              // Transform them into body frame from the geometry-centric frame
              points = element->getLocalTransform() * points;

              all_vertices.conservativeResize(3, points.cols() + all_vertices.cols());
              all_vertices.block(0, all_vertices.cols() - points.cols(), 3, points.cols()) = points;
              DrakeShapes::TrianglesVector faces;
              geometry.getFaces(&faces);
              // Also use this loop to offset the vertex indices to the appropriate all_vertices indices.
              for (auto& face : faces) {
                face[0] += (all_vertices.cols() - points.cols());
                face[1] += (all_vertices.cols() - points.cols());
                face[2] += (all_vertices.cols() - points.cols());
                face_body_map.push_back( body_i );
              }
              all_faces.insert(all_faces.end(), faces.begin(), faces.end());
            } 
          }
        }
      }

      // And reform those extracted verticies and meshes into
      // matrices for our optimization
      MatrixXd F(all_faces.size(), all_vertices.cols());
      // Don't include the "world" body
      MatrixXd B(robot_.get_num_bodies() - 1, all_faces.size());
      B.setZero();
      F.setZero();
      for (int i=0; i<all_faces.size(); i++){
        // Generate sub-block of face selection matrix F
        F(i, all_faces[i][0]) = 1.;
        F(i, all_faces[i][1]) = 1.;
        F(i, all_faces[i][2]) = 1.;
        // Don't include the "world" body.
        B(face_body_map[i]-1, i) = 1.0;
      }

      // See https://www.sharelatex.com/project/5850590c38884b7c6f6aedd1
      // for problem formulation
      MathematicalProgram prog;

      // Allocate slacks to choose minimum L-1 norm over objects
      auto phi = prog.NewContinuousVariables(scene_pts.cols(), 1, "phi");
      
      // And slacks to store term-wise absolute value terms for L-1 norm calculation
      auto alpha = prog.NewContinuousVariables(3, scene_pts.cols(), "alpha");

      // Each row is a set of affine coefficients relating the scene point to a combination
      // of vertices on a single face of the model
      auto C = prog.NewContinuousVariables(scene_pts.cols(), all_vertices.cols(), "C");
      // Binary variable selects which face is being corresponded to
      auto f = prog.NewBinaryVariables(scene_pts.cols(), F.rows(),"f");

      struct TransformationVars {
        VectorDecisionVariable<3> T;
        MatrixDecisionVariable<3,3> R;
      };
      std::vector<TransformationVars> transform_by_object;
      for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){
        TransformationVars new_tr;
        char name_postfix[100];
        sprintf(name_postfix, "_%s_%d", robot_.get_body(body_i).get_model_name().c_str(), body_i);
        new_tr.T = prog.NewContinuousVariables<3>(string("T")+string(name_postfix));
        prog.AddBoundingBoxConstraint(-100*VectorXd::Ones(3), 100*VectorXd::Ones(3), new_tr.T);
        new_tr.R = NewRotationMatrixVars(&prog, string("R") + string(name_postfix));

        if (optRotationConstraint_ > 0){
          switch (optRotationConstraint_){
            case 1:
              break;
            case 2:
              // Columnwise and row-wise L1-norm >=1 constraints
              for (int k=0; k<3; k++){
                prog.AddLinearConstraint(Vector3d::Ones().transpose(), 1.0, std::numeric_limits<double>::infinity(), new_tr.R.row(k).transpose());
                prog.AddLinearConstraint(Vector3d::Ones().transpose(), 1.0, std::numeric_limits<double>::infinity(), new_tr.R.col(k));
              }
              break;
            case 3:
              addMcCormickQuaternionConstraint(prog, new_tr.R, optRotationConstraintNumFaces_, optRotationConstraintNumFaces_);
              break;
            case 4:
              AddRotationMatrixMcCormickEnvelopeMilpConstraints(&prog, new_tr.R, optRotationConstraintNumFaces_);
              break;
            case 5:
              AddBoundingBoxConstraintsImpliedByRollPitchYawLimits(&prog, new_tr.R, kYaw_0_to_PI_2 | kPitch_0_to_PI_2 | kRoll_0_to_PI_2);
              break;
            default:
              printf("invalid optRotationConstraint_ option!\n");
              exit(-1);
              break;
          }
        } else {
          // constrain rotations to ground truth
          auto ground_truth_tf = robot_.relativeTransform(robot_kinematics_cache, robot_.get_body(body_i).get_body_index(), 0);
          // Formulas only work for vectors, so implement this constraint column-wise
          for (int i = 0; i < 3; ++i) {
            prog.AddLinearEqualityConstraint(new_tr.R.col(i) - ground_truth_tf.rotation().col(i), Eigen::Vector3d::Zero());
          }
        }
        transform_by_object.push_back(new_tr);
      }

      // Optimization pushes on slacks to make them tight (make them do their job)
      prog.AddLinearCost(1.0 * VectorXd::Ones(scene_pts.cols()), phi);
      /*
      for (int k=0; k<3; k++){
        prog.AddLinearCost(1.0 * VectorXd::Ones(alpha.cols()), {alpha.row(k)});
      }./bias
      */

      // Constrain slacks nonnegative, to help the estimation of lower bound in relaxation  
      prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), phi);
      for (int k=0; k<3; k++){
        prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), {alpha.row(k).transpose()});
      }

      // Constrain each row of C to sum to 1 if a face is selected, to make them proper
      // affine coefficients
      Eigen::MatrixXd C1 = Eigen::MatrixXd::Ones(1, C.cols()+f.cols());
      // sum(C_i) = sum(f_i)
      // sum(C_i) - sum(f_i) = 0
      C1.block(0, C.cols(), 1, f.cols()) = -Eigen::MatrixXd::Ones(1, f.cols());
      for (size_t k=0; k<C.rows(); k++){
        prog.AddLinearEqualityConstraint(C1, 0, {C.row(k).transpose(), f.row(k).transpose()});
      }

      // Constrain each row of f to sum to 1, to force selection of exactly
      // one face to correspond to
      Eigen::MatrixXd f1 = Eigen::MatrixXd::Ones(1, f.cols());
      for (size_t k=0; k<f.rows(); k++){
        if (optAllowOutliers_){
          prog.AddLinearConstraint(f1, 0, 1, f.row(k).transpose()); 
        } else {
          prog.AddLinearEqualityConstraint(f1, 1, f.row(k).transpose());
        }
      }

      // Force all elems of C nonnegative
      for (int i=0; i<C.rows(); i++){
        for (int j=0; j<C.cols(); j++){
          prog.AddBoundingBoxConstraint(0.0, 1.0, C(i, j));
        }
      }

      // Force elems of C to be zero unless their corresponding vertex is a member
      // of an active face
      // That is,
      //   C_{i, j} <= F_{:, j}^T * f_{i, :}^T
      // or reorganized
      // [0] <= [F_{:, j}^T -1] [f_{i, :}^T C_{i, j}]
      //         
      for (int i=0; i<C.rows(); i++){
        for (int j=0; j<C.cols(); j++){
          MatrixXd A(1, F.rows() + 1);
          A.block(0, 0, 1, F.rows()) = F.col(j).transpose();
          A(0, F.rows()) = -1.0;

          prog.AddLinearConstraint(A, 0.0, std::numeric_limits<double>::infinity(), {f.row(i).transpose(), C.block<1,1>(i,j)});
        }
      }

      // I'm adding a dummy var constrained to zero to 
      // fill out the diagonals of C_i.
      auto C_dummy = prog.NewContinuousVariables(1, "c_dummy_zero");
      prog.AddLinearEqualityConstraint(Eigen::MatrixXd::Ones(1, 1), Eigen::MatrixXd::Zero(1, 1), C_dummy);

      // Helper variable to produce linear constraint
      // alpha_{i, l} +/- (R_l * s_i + T - M C_{i, :}^T) - Big * B_l * f_i >= -Big
      auto AlphaConstrPos = Eigen::RowVectorXd(1, 1+3+1+all_vertices.cols() + B.cols());    
      AlphaConstrPos.block<1, 1>(0, 0) = MatrixXd::Ones(1, 1); // multiplies alpha_{i, l} elem
      AlphaConstrPos.block<1, 1>(0, 4) = -1.0 * MatrixXd::Ones(1, 1); // T bias term
      auto AlphaConstrNeg = Eigen::RowVectorXd(1, 1+3+1+all_vertices.cols() + B.cols());    
      AlphaConstrNeg.block<1, 1>(0, 0) = MatrixXd::Ones(1, 1); // multiplies alpha_{i, l} elem
      AlphaConstrNeg.block<1, 1>(0, 4) = MatrixXd::Ones(1, 1); // T bias term

      printf("Starting to add correspondence costs... ");
      for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){
        AlphaConstrPos.block(0, 5+all_vertices.cols(), 1, B.cols()) = -kBigNumber*B.row(body_i-1); // multiplies f_i
        AlphaConstrNeg.block(0, 5+all_vertices.cols(), 1, B.cols()) = -kBigNumber*B.row(body_i-1); // multiplies f_i

        for (int i=0; i<scene_pts.cols(); i++){
          printf("\r\tgenerating guess for body (%d)/(%d), point (%d)/(%d)", body_i, (int)robot_.get_num_bodies(), i, (int)scene_pts.cols());

          // constrain L-1 distance slack based on correspondences
          // phi_i >= 1^T alpha_{i}
          // phi_i - 1&T alpha_{i} >= 0
          RowVectorXd PhiConstr(1 + 3);
          PhiConstr.setZero();
          PhiConstr(0, 0) = 1.0; // multiplies phi
          PhiConstr.block<1,3>(0,1) = -RowVector3d::Ones(); // multiplies alpha
          prog.AddLinearConstraint(PhiConstr, 0, std::numeric_limits<double>::infinity(),
          {phi.block<1,1>(i, 0),
           alpha.col(i)});

          // If we're allowing outliers, we need to constrain each phi_i to be bigger than
          // a penalty amount if no faces are selected
          if (optAllowOutliers_){
            // phi_i >= phi_max - (ones * f_i)*BIG
            // -> phi_ + (ones * f_i)*BIG >= phi_max
            Eigen::MatrixXd A_phimax = Eigen::MatrixXd::Ones(1, f.cols() + 1);
            A_phimax.block(0, 0, 1, f.cols()) = Eigen::MatrixXd::Ones(1, f.cols())*kBigNumber;
            for (size_t k=0; k<f.rows(); k++){
              prog.AddLinearConstraint(A_phimax, optPhiMax_, std::numeric_limits<double>::infinity(), {f.row(k).transpose(), phi.row(k)});
            }
          }

          // Alphaconstr, containing the scene and model points and a translation bias term, is used the constraints
          // on the three elems of alpha_{i, l}
          auto s_xyz = scene_pts.col(i);
          AlphaConstrPos.block<1, 3>(0, 1) = -s_xyz.transpose(); // Multiples R
          AlphaConstrNeg.block<1, 3>(0, 1) = s_xyz.transpose(); // Multiples R

          AlphaConstrPos.block(0, 5, 1, all_vertices.cols()) = 1.0 * all_vertices.row(0); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrPos, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(0, i),
             transform_by_object[body_i-1].R.block<1, 3>(0, 0).transpose(), 
             transform_by_object[body_i-1].T.block<1,1>(0,0),
             C.row(i).transpose(),
             f.row(i).transpose()});

          AlphaConstrPos.block(0, 5, 1, all_vertices.cols()) = 1.0 * all_vertices.row(1); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrPos, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(1, i),
             transform_by_object[body_i-1].R.block<1, 3>(1, 0).transpose(), 
             transform_by_object[body_i-1].T.block<1,1>(1,0),
             C.row(i).transpose(),
             f.row(i).transpose()});

          AlphaConstrPos.block(0, 5, 1, all_vertices.cols()) = 1.0 * all_vertices.row(2); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrPos, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(2, i),
             transform_by_object[body_i-1].R.block<1, 3>(2, 0).transpose(), 
             transform_by_object[body_i-1].T.block<1,1>(2,0),
             C.row(i).transpose(),
             f.row(i).transpose()});

          AlphaConstrNeg.block(0, 5, 1, all_vertices.cols()) = -1.0 * all_vertices.row(0); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrNeg, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(0, i),
             transform_by_object[body_i-1].R.block<1, 3>(0, 0).transpose(), 
             transform_by_object[body_i-1].T.block<1,1>(0,0),
             C.row(i).transpose(),
             f.row(i).transpose()});
          AlphaConstrNeg.block(0, 5, 1, all_vertices.cols()) = -1.0 * all_vertices.row(1); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrNeg, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(1, i),
             transform_by_object[body_i-1].R.block<1, 3>(1, 0).transpose(), 
             transform_by_object[body_i-1].T.block<1,1>(1,0),
             C.row(i).transpose(),
             f.row(i).transpose()});
          AlphaConstrNeg.block(0, 5, 1, all_vertices.cols()) = -1.0 * all_vertices.row(2); // multiplies the selection vars
          prog.AddLinearConstraint(AlphaConstrNeg, -kBigNumber, std::numeric_limits<double>::infinity(),
            {alpha.block<1,1>(2, i),
             transform_by_object[body_i-1].R.block<1, 3>(2, 0).transpose(), 
             transform_by_object[body_i-1].T.block<1,1>(2,0),
             C.row(i).transpose(),
             f.row(i).transpose()});
        }
      }
      printf("\n");

      /*
      printf("Starting to add joint constraints...\n");
      for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){
        const auto& body = robot_.get_body(body_i);
        if (body.has_joint() && typeid(body.getJoint()) == typeid(RevoluteJoint)){
          auto revolute_joint = static_cast<const RevoluteJoint&>(body.getJoint());
          Vector3d rot_axis = revolute_joint.rotation_axis();

          int parent_id = body.get_parent()->get_body_index();
          // I'm going to assume that body_id and parent_id are good and not references
          // to world, as nothing should be revolute-jointed to this world... right?

          // To enforce a pin joint between two bodies described in maximal coordinates
          // by (R_a, T_a) and (R_b, T_b), we take two points on the rotation axis
          // in each bodies axis:
          // p^a_1, p^a_2 (in a's frame)
          // p^b_1, p^b_2 (in b's frame)
          // and constrain them to be the same in global coordinates:
          // R_a * p^a_1 + T_a = R_b * p^b_1 + T_b
          // R_a * p^a_2 + T_a = R_b * p^b_2 + T_b

          // For one point, we take the origin in the child body
          Vector3d p_a_1 = Vector3d::Zero();
          // And for the other, we take a unit length along the rotation axis
          Vector3d p_a_2 = rot_axis;
          // Transform them into the parent body's frame
          // (the kinematics cache is used here is not important and shouldn't affect results)
          Vector3d p_b_1 = robot_.transformPoints(robot_kinematics_cache, p_a_1, body_i, parent_id);
          Vector3d p_b_2 = robot_.transformPoints(robot_kinematics_cache, p_a_2, body_i, parent_id);

          // Unfortunately, our decision variable-encoded transforms actually go in the other direction -- 
          // they transform from global frame to body frame. So we implement this constrain with
          // one layer of indirection:
          // R_a * z_i + T_a = p_a_i   for z_i = unconstrained 3x1 decision variable matrix
          // R_b * z_i + T_b = p_b_i           fuck me that isn't linear

          auto z_a = prog.NewContinuousVariables<3>("z_1");
          auto z_b = prog.NewContinuousVariables<3>("z_2");

          auto e1_a = transform_by_object[body_i-1].R * z_a + transform_by_object[body_i-1].T;
          auto e1_b = transform_by_object[body_i-1].R * z_b + transform_by_object[body_i-1].T;
          auto e2_a = transform_by_object[parent_id-1].R * z_a + transform_by_object[parent_id-1].T;
          auto e2_b = transform_by_object[parent_id-1].R * z_b + transform_by_object[parent_id-1].T;

          prog.AddLinearEqualityConstraint(e1_a, p_a_1);
          prog.AddLinearEqualityConstraint(e1_b, p_b_1);
          prog.AddLinearEqualityConstraint(e2_a, p_a_2);
          prog.AddLinearEqualityConstraint(e2_b, p_b_2);
        }
      }
      */

      double now = getUnixTime();

      GurobiSolver gurobi_solver;
      MosekSolver mosek_solver;

      prog.SetSolverOption(SolverType::kGurobi, "OutputFlag", 1);
      prog.SetSolverOption(SolverType::kGurobi, "LogToConsole", 1);
      prog.SetSolverOption(SolverType::kGurobi, "LogFile", "loggg.gur");
      prog.SetSolverOption(SolverType::kGurobi, "DisplayInterval", 5);

      if (config_["gurobi_int_options"]){
        for (auto iter = config_["gurobi_int_options"].begin();
             iter != config_["gurobi_int_options"].end();
             iter++){
          prog.SetSolverOption(SolverType::kGurobi, iter->first.as<string>(), iter->second.as<int>());
        }
      }
      if (config_["gurobi_float_options"]){
        for (auto iter = config_["gurobi_float_options"].begin();
             iter != config_["gurobi_float_options"].end();
             iter++){
          prog.SetSolverOption(SolverType::kGurobi, iter->first.as<string>(), iter->second.as<float>());
        }
      }

      if (optUseInitialGuess_){
        // Corruption should be done from q_gt, then recover maximal coordinates from transform queries,
        // get mesh and do projection using get_closest_points in drake 
        VectorXd corruption_vec(q_robot_gt_.rows()); 
        for (int i=0; i<corruption_vec.rows(); i++){
          corruption_vec[i] = randrange(-optCorruption_, optCorruption_);
        }
        VectorXd q_robot_corrupt = q_robot_gt_ + corruption_vec;
        KinematicsCache<double> robot_kinematics_cache_corrupt = robot_.doKinematics(q_robot_corrupt);
        cout << "q robot corrupt " << q_robot_corrupt.transpose() << endl;

        for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){
          auto tf = robot_.relativeTransform(robot_kinematics_cache_corrupt, body_i, 0);
          //prog.SetInitialGuess(transform_by_object[body_i-1].T, tf.translation());
          //prog.SetInitialGuess(transform_by_object[body_i-1].R, tf.matrix().block<3, 3>(0, 0));
        }

        // for every scene point, project it down onto the models at the supplied TF to get closest object, and use 
        // that face assignment as our guess if the distance isn't too great
        VectorXd search_phi;
        Matrix3Xd search_norm;
        Matrix3Xd search_x;
        Matrix3Xd search_body_x;
        vector<int> search_body_idx;
        robot_.collisionDetectFromPoints(robot_kinematics_cache_corrupt, scene_pts, search_phi,
                    search_norm, search_x, search_body_x, search_body_idx, false);

        MatrixXd f0(scene_pts.cols(), F.rows());
        f0.setZero();
        printf("Starting to backsolve initial guess...\n");
        for (int i=0; i<scene_pts.cols(); i++){
          printf("\r\tgenerating guess for point (%d)/(%d)", i, (int)scene_pts.cols());
          // Find the face it's closest to on body i
          double dist = std::numeric_limits<double>::infinity();
          int face_ind = 0;
          Vector3d closest_pt;
          for (int j=0; j<all_faces.size(); j++){
            if (search_body_idx[i] > 0 && B(search_body_idx[i]-1, j) > 0.5){
              // We're looking for argmin_{pt_proj} ||pt - pt_proj||
              //   such that pt_proj = verts * C
              MathematicalProgram prog_proj;
              auto pt_proj = prog_proj.NewContinuousVariables(3, 1, "pt_proj");

              // pt_proj is an affine combination of vertices
              Matrix3Xd verts(3, 3);
              for (int k=0; k<3; k++){
                verts.col(k) = all_vertices.col(all_faces[j][k]);
              }
              verts = robot_.transformPoints(robot_kinematics_cache_corrupt, verts, search_body_idx[i], 0);
              auto C = prog_proj.NewContinuousVariables(3, 1, "C");
              prog_proj.AddBoundingBoxConstraint(0.0, 1.0, C);
              prog_proj.AddLinearEqualityConstraint(VectorXd::Ones(3).transpose(), 1.0, C);
              Matrix3Xd A(3, 3 + 3);
              A.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
              A.block(0, 3, 3, 3) = -verts;
              prog_proj.AddLinearEqualityConstraint(A, Vector3d::Zero(), {pt_proj, C});

              // minimize quadratic error between our pt and the projected point
              prog_proj.AddL2NormCost(MatrixXd::Identity(3, 3), scene_pts.col(i), pt_proj);

              auto out = prog_proj.Solve();

              if (out >= 0){
                Vector3d pt_proj_sol = prog_proj.GetSolution(pt_proj);
                double new_dist = (pt_proj_sol - scene_pts.col(i)).norm();
                if (new_dist < dist){
                  dist = new_dist;
                  face_ind = j;
                  closest_pt = pt_proj_sol;
                }
              } else {
                printf("optimization returned negative number? that should have been feasible!\n");
              }
            }
          }
          if (dist < 0.1){
            f0(i, face_ind) = 1;
          }
          // else it's an outlier point
        }
        prog.SetInitialGuess(f, f0);
      }

      //  prog.SetSolverOption(SolverType::kGurobi, "Cutoff", 50.0);
      // isn't doing anything... not invoking this tool right?
      //  prog.SetSolverOption(SolverType::kGurobi, "TuneJobs", 8);
      //  prog.SetSolverOption(SolverType::kGurobi, "TuneResults", 3);
      //prog.SetSolverOption(SolverType::kGurobi, )

      auto out = gurobi_solver.Solve(prog);
      string problem_string = "rigidtf";
      double elapsed = getUnixTime() - now;

      //prog.PrintSolution();
      printf("Code %d, problem %s solved for %lu scene solved in: %f\n", out, problem_string.c_str(), scene_pts.cols(), elapsed);

      std::vector<Solution> solutions;

      // for (int sol_i = 0; sol_i < prog.get_num_solutions(); sol_i++) {
      for (int sol_i = 0; sol_i < 1; sol_i ++){
        printf("==================================================\n");
        printf("======================SOL %d ======================\n", sol_i);
        printf("==================================================\n");
        auto f_est= prog.GetSolution(f); //, sol_i);
        auto C_est = prog.GetSolution(C); //, sol_i);

        Solution new_solution;

        for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++){
          const RigidBody<double>& body = robot_.get_body(body_i);

          ObjectDetection new_detection;
          new_detection.obj_ind = body_i;
      
          printf("************************************************\n");
          printf("Concerning model %d (%s):\n", body_i, body.get_name().c_str());
          printf("------------------------------------------------\n");
          Vector3d Tf = prog.GetSolution(transform_by_object[body_i-1].T); //, sol_i);
          Matrix3d Rf = prog.GetSolution(transform_by_object[body_i-1].R); //, sol_i);
          printf("Transform:\n");
          printf("\tTranslation: %f, %f, %f\n", Tf(0, 0), Tf(1, 0), Tf(2, 0));
          printf("\tRotation:\n");
          printf("\t\t%f, %f, %f\n", Rf(0, 0), Rf(0, 1), Rf(0, 2));
          printf("\t\t%f, %f, %f\n", Rf(1, 0), Rf(1, 1), Rf(1, 2));
          printf("\t\t%f, %f, %f\n", Rf(2, 0), Rf(2, 1), Rf(2, 2));
          printf("------------------------------------------------\n");
          printf("************************************************\n");
          new_detection.est_tf.setIdentity();
          new_detection.est_tf.translation() = Tf;
          new_detection.est_tf.matrix().block<3,3>(0,0) = Rf;
          // And flip it, so that it transforms from world -> model
          new_detection.est_tf = new_detection.est_tf.inverse();

          for (int scene_i=0; scene_i<scene_pts.cols(); scene_i++){
            for (int face_i=0; face_i<f_est.cols(); face_i++){
              // if this face is assigned, and this face is a member of this object,
              // then display this point
              if (f_est(scene_i, face_i) > 0.5 && B(body_i-1, face_i) > 0.5){
                PointCorrespondence new_corresp;
                new_corresp.scene_pt = scene_pts.col(scene_i);
                new_corresp.model_pt = new_detection.est_tf * scene_pts.col(scene_i);
                new_corresp.scene_ind = scene_i;
                new_corresp.face_ind = face_i;
                for (int k_v=0; k_v<all_vertices.cols(); k_v++){
                  if (C_est(scene_i, k_v) >= 0.0){
                    new_corresp.model_verts.push_back( all_vertices.col(k_v) );
                    new_corresp.vert_weights.push_back( C_est(scene_i, k_v) );
                    new_corresp.vert_inds.push_back(k_v);
                  }
                }
                new_detection.correspondences.push_back(new_corresp);
              }
            }
          }

          // Include this as a detection if we have correspondences
          // to support it.
          if (new_detection.correspondences.size() > 0)
            new_solution.detections.push_back(new_detection);
        }

        new_solution.objective = prog.GetSolution(phi).sum(); //, sol_i).sum();
        solutions.push_back(new_solution);
      }

      return solutions;
    }

    RigidBodyTree<double> & get_robot() {
      return robot_;
    }

  private:
    RigidBodyTree<double> robot_;
    VectorXd q_robot_gt_;

    YAML::Node config_;

    int optNumRays_ = 10;
    int optRotationConstraint_ = 4;
    int optRotationConstraintNumFaces_ = 2;
    bool optAllowOutliers_ = true;
    double optPhiMax_ = 0.1;
    bool optUseInitialGuess_ = false;
    double optCorruption_ = 100.0;

};

int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc != 2){
    printf("Use: milp_multiple_mesh_models_detector_drake <config file>\n");
    exit(-1);
  }

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "MILP Multiple Mesh Models Detector, Drake Ver, " << asctime(curtime);
  cout << "Config file " << string(argv[1]) << endl;
  cout << "***************************" << endl << endl;


  // Bring in config file
  string yamlString = string(argv[1]);
  YAML::Node config = YAML::LoadFile(yamlString);

  // Set up a point cloud generator
  if (config["point_cloud_generator_options"] == NULL){
    runtime_error("Config needs a point cloud generator option set.");
  }
  PointCloudGenerator pcg(config["point_cloud_generator_options"]);

  // Generate a point set
  Matrix3Xd scene_pts = pcg.samplePointCloudFromSurface();
  
  // Visualize the results using the drake visualizer.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts, {"scene_pts"});
  rm.publishRigidBodyTree(pcg.get_robot(), pcg.get_q_robot(), Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});

  // And set up our detector
  if (config["detector_options"] == NULL){
    runtime_error("Config needs a detector option set.");
  }
  MILPMultipleMeshModelDetector detector(config["detector_options"]);
  auto solutions = detector.doObjectDetection(scene_pts);

  for (const auto& solution : solutions){
    printf("printing sol\n");
    stringstream sol_name;
    sol_name << "sol_obj_" << solution.objective;

    for (const auto& detection : solution.detections){
      const RigidBody<double>& body = detector.get_robot().get_body(detection.obj_ind);

      for (const auto& collision_elem_id : body.get_collision_element_ids()) {
        stringstream collision_elem_id_str;
        collision_elem_id_str << collision_elem_id;
        auto element = detector.get_robot().FindCollisionElement(collision_elem_id);
        if (element->hasGeometry()){
          vector<string> path;
          path.push_back(sol_name.str());
          path.push_back(body.get_name());
          path.push_back(collision_elem_id_str.str());
          rm.publishGeometry(element->getGeometry(), detection.est_tf * element->getLocalTransform(), Vector4d(0.3, 1.0, 0.3, 0.2), path);
        }
      }
    }
  }

  return 0;
}
