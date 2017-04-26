#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <unistd.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"

#include "gurobi_c++.h"

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"
#include "optimization_helpers.h"
#include "rotation_helpers.h"

#include "RemoteTreeViewerWrapper.hpp"
#include "miqp_mesh_model_detector.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;
using namespace drake::systems;
using namespace drake::symbolic;
using namespace drake::solvers;
using namespace drake::math;

const double kMaxConsideredICPDistance = 0.5;

int done = 0;
void callICPProcessingForever(MIQPMultipleMeshModelDetector * detector){
  while (!done){
    detector->doICPProcessing();
    usleep(1000*1000*5);
  }
}

// Adapted from https://www.gamedev.net/topic/552906-closest-point-on-triangle/
// and converted to Eigen by gizatt@mit.edu
Vector3d closesPointOnTriangle( const vector<Vector3d>& triangle, const Vector3d& sourcePosition )
{
    Vector3d edge0 = triangle[1] - triangle[0];
    Vector3d edge1 = triangle[2] - triangle[0];
    Vector3d v0 = triangle[0] - sourcePosition;

    float a = edge0.transpose() * edge0;
    float b = edge0.transpose() * edge1;
    float c = edge1.transpose() * edge1;
    float d = edge0.transpose() * v0;
    float e = edge1.transpose() * v0;

    float det = a*c - b*b;
    float s = b*e - c*d;
    float t = b*d - a*e;

    if ( s + t < det )
    {
        if ( s < 0.f )
        {
            if ( t < 0.f )
            {
                if ( d < 0.f )
                {
                    s = clamp( -d/a, 0.f, 1.f );
                    t = 0.f;
                }
                else
                {
                    s = 0.f;
                    t = clamp( -e/c, 0.f, 1.f );
                }
            }
            else
            {
                s = 0.f;
                t = clamp( -e/c, 0.f, 1.f );
            }
        }
        else if ( t < 0.f )
        {
            s = clamp( -d/a, 0.f, 1.f );
            t = 0.f;
        }
        else
        {
            float invDet = 1.f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if ( s < 0.f )
        {
            float tmp0 = b+d;
            float tmp1 = c+e;
            if ( tmp1 > tmp0 )
            {
                float numer = tmp1 - tmp0;
                float denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                t = clamp( -e/c, 0.f, 1.f );
                s = 0.f;
            }
        }
        else if ( t < 0.f )
        {
            if ( a+d > b+e )
            {
                float numer = c+e-b-d;
                float denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                s = clamp( -e/c, 0.f, 1.f );
                t = 0.f;
            }
        }
        else
        {
            float numer = c+e-b-d;
            float denom = a-2*b+c;
            s = clamp( numer/denom, 0.f, 1.f );
            t = 1.f - s;
        }
    }

    return triangle[0] + s * edge0 + t * edge1;
}


void mipSolCallbackFunction(const MathematicalProgram& prog, const drake::solvers::GurobiSolver::SolveStatusInfo& solve_info, void * usrdata){
  ((MIQPMultipleMeshModelDetector *)usrdata)->handleMipSolCallbackFunction(prog, solve_info);
}
void MIQPMultipleMeshModelDetector::handleMipSolCallbackFunction(const MathematicalProgram& prog, const drake::solvers::GurobiSolver::SolveStatusInfo& solve_info){
  // Extract current solution
  VectorXd q_robot(robot_.get_num_positions());
  for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++){
    const RigidBody<double>& body = robot_.get_body(body_i);

    Vector3d Tf = prog.GetSolution(transform_by_object_[body_i-1].T); //, sol_i);
    Matrix3d Rf = prog.GetSolution(transform_by_object_[body_i-1].R); //, sol_i);

    Affine3d est_tf;
    est_tf.setIdentity();
    est_tf.translation() = Tf;
    est_tf.matrix().block<3,3>(0,0) = Rf;

    // And flip it, so that it transforms from world -> model
    est_tf = est_tf.inverse();

    // TODO(gizatt) This state vector reconstruction assumes all bodies
    // have floating bases. Reconstructing joint states will require
    // more clever and careful optimization...

    q_robot.block<3, 1>(6*(body_i - 1), 0) = est_tf.translation();
    q_robot.block<3, 1>(6*(body_i - 1) + 3, 0) = rotmat2rpy(est_tf.rotation());
  }

  if (getUnixTime() - last_published_sol_ > 0.05 && q_robot.transpose() * q_robot < 100.0){
    last_published_sol_ = getUnixTime();
    RemoteTreeViewerWrapper rm;
    rm.publishRigidBodyTree(robot_, q_robot, Vector4d(0.2, 0.5, 1.0, 0.5), {"latest_sol"});
  }

  if (solve_info.current_objective < best_sol_objective_yet_){
    printf("Best sol objective yet: %f\n", solve_info.current_objective);
    best_sol_objective_yet_ = solve_info.current_objective;
    RemoteTreeViewerWrapper rm;
    rm.publishRigidBodyTree(robot_, q_robot, Vector4d(0.0, 0.8, 0.8, 0.5), {"incumbent_sol"});
  }

  if (optUseICPHeuristic_){
    icp_search_seeds_lock_.lock();
    icp_search_seeds_.push(q_robot);
    icp_search_seeds_lock_.unlock();
  }

  // Record all feasible nodes, as these aren't frequent
  solve_history.push_back(
    MIQPMultipleMeshModelDetector::SolveHistoryElem({getUnixTime(),
      solve_info.reported_runtime,
      solve_info.best_objective,
      solve_info.best_bound,
      solve_info.explored_node_count,
      solve_info.feasible_solutions_count
    })
  );
  return;
}

void mipNodeCallbackFunction(const MathematicalProgram& prog, const drake::solvers::GurobiSolver::SolveStatusInfo& solve_info, void * usrdata, VectorXd& vals, VectorXDecisionVariable& vars){
  ((MIQPMultipleMeshModelDetector *)usrdata)->handleMipNodeCallbackFunction(prog, solve_info, vals, vars);
}
void MIQPMultipleMeshModelDetector::handleMipNodeCallbackFunction(const MathematicalProgram& prog, const drake::solvers::GurobiSolver::SolveStatusInfo& solve_info, VectorXd& vals, VectorXDecisionVariable& vars){
  // Extract current (relaxed!) solution
  VectorXd q_robot(robot_.get_num_positions());
  for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++){
    const RigidBody<double>& body = robot_.get_body(body_i);

    Vector3d Tf = prog.GetSolution(transform_by_object_[body_i-1].T); //, sol_i);
    Matrix3d Rf = prog.GetSolution(transform_by_object_[body_i-1].R); //, sol_i);

    Affine3d est_tf;
    est_tf.setIdentity();
    est_tf.translation() = Tf;
    est_tf.matrix().block<3,3>(0,0) = Rf;

    // And flip it, so that it transforms from world -> model
    est_tf = est_tf.inverse();


    // TODO(gizatt) This state vector reconstruction assumes all bodies
    // have floating bases. Reconstructing joint states will require
    // more clever and careful optimization...

    q_robot.block<3, 1>(6*(body_i - 1), 0) = est_tf.translation();
    q_robot.block<3, 1>(6*(body_i - 1) + 3, 0) = rotmat2rpy(est_tf.rotation());
  }

  if (is_finite(q_robot)){
    if (getUnixTime() - last_published_node_ > 0.1 && (q_robot.transpose() * q_robot) < 1000.) {
      last_published_node_ = getUnixTime();
      RemoteTreeViewerWrapper rm;
      rm.publishRigidBodyTree(robot_, q_robot, Vector4d(0.2, 0.2, 1.0, 0.3), {"latest_node"});
    }

    // Try to ICP on this one
    if (optUseICPHeuristic_){
      icp_search_seeds_lock_.lock();
      if (icp_search_seeds_.size() < 1)
        icp_search_seeds_.push(q_robot);
      icp_search_seeds_lock_.unlock();
    }
  }

  // And add a heuristic sol from our queue, if we have one available.
  // (This will likely be unrelated to the current node.)
  if (new_heuristic_sols_.size() > 0){
    if (optUseICPHeuristic_){
      new_heuristic_sols_lock_.lock();
      vals = new_heuristic_sols_.front().vals;
      vars = new_heuristic_sols_.front().vars;
      new_heuristic_sols_.pop_front();
      new_heuristic_sols_lock_.unlock();
    }
  }


  // Record solve info once every 0.05 seconds at most frequent
  if (solve_history.size() == 0 || getUnixTime() - solve_history[solve_history.size() - 1].wall_time > 0.1){
    solve_history.push_back(
        MIQPMultipleMeshModelDetector::SolveHistoryElem({getUnixTime(),
          solve_info.reported_runtime,
          solve_info.best_objective,
          solve_info.best_bound,
          solve_info.explored_node_count,
          solve_info.feasible_solutions_count
        })
      );
  }
  return;
}

void MIQPMultipleMeshModelDetector::doICPProcessing(){
  RemoteTreeViewerWrapper rm;
  VectorXd q_robot;
  icp_search_seeds_lock_.lock();
  if (icp_search_seeds_.size() > 0){
    q_robot = icp_search_seeds_.top();
    icp_search_seeds_.pop();
  }
  icp_search_seeds_lock_.unlock();

  if (q_robot.size() == 0){
    return;
  } else if (q_robot.size() != robot_.get_num_positions()){
    printf("Got %ld positions in q_robot in icp procesing, but need %d. ???\n", q_robot.size(), robot_.get_num_positions());
    return;
  }

  // Perform steps of ICP on heuristic solution
  // TODO(gizatt) This code is pretty dirty, upgrade to MathematicalProgram
  // and tidy up. Maybe make this a generic method in Drake to call as a utility
  // given an RBT and a point cloud? That'd be super useful!
  int consecutive_rounds_of_nondecreasing_error = 0;
  double last_error = scene_pts_.size() * optPhiMax_;
  double last_published_icp = getUnixTime() - 100;
  for (int icp_iter=0; icp_iter<optICPIters_; icp_iter++) {
    KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot);
    
    // So, for now, the optimization problem is:
    // 0.5 * x.' Q x + f.' x
    // and since we're unconstrained then solve as linear system
    // Qx = -f
    int nq = q_robot.rows();
    VectorXd f(nq);
    f.setZero();
    MatrixXd Q(nq, nq);
    Q.setZero();
    double K = 0.;

    // Prior for ICP steps will be the last pose
    Q += optICPPriorWeight_ * MatrixXd::Identity(nq, nq);
    f -= q_robot * optICPPriorWeight_;
    K += 2 * q_robot.transpose() * optICPPriorWeight_ * q_robot;


    // Search closest points
    VectorXd phi;
    Matrix3Xd normal;
    Matrix3Xd x;
    Matrix3Xd body_x;
    vector<int> body_idx;
    robot_.collisionDetectFromPoints(robot_kinematics_cache, scene_pts_, phi,
                normal, x, body_x, body_idx, false);

    // Get error and decide whether to terminate early
    double error = 0.0;
    for (int i = 0; i < phi.rows(); i++){
      error += phi[i];
    } 
    if (error == 0.0){
      // Precisely zero error, so we're unlikely to improve, either because
      // we're lost, or we're perfect
      break;
    } else if (error > (last_error - 0.0001) ){
      consecutive_rounds_of_nondecreasing_error++;
      if (consecutive_rounds_of_nondecreasing_error > 10)
        break;
    } else {
      consecutive_rounds_of_nondecreasing_error = 0;
    }
    last_error = error;

    // Prepare to group per body, since we'll get major performance gains
    // by doing bulk forward kinematic transforms on many points corresponding to
    // a single body at once.
    std::vector<int> num_points_on_body(robot_.get_num_bodies(), 0);
    for (int i=0; i < body_idx.size(); i++)
      if (body_idx[i] >= 0)
        num_points_on_body[body_idx[i]] += 1;

    // For every body...
    for (int i=0; i < robot_.get_num_bodies(); i++){
      if (num_points_on_body[i] > 0){
        // Collect results from raycast that correspond to this body.
        Matrix3Xd z(3, num_points_on_body[i]); // points, in world frame, near this body
        Matrix3Xd z_prime(3, num_points_on_body[i]); // same points projected onto surface of body
        Matrix3Xd body_z_prime(3, num_points_on_body[i]); // projected points in body frame
        Matrix3Xd z_norms(3, num_points_on_body[i]); // normals corresponding to these points
        int k = 0;
        
        // First collect average distance to nearby points
        double avg_dist = 0.0;
        for (int j=0; j < body_idx.size(); j++){
          assert(k < body_idx.size());
          if (body_idx[j] == i){
            assert(j < scene_pts_.cols());
            if (scene_pts_(0, j) == 0.0){
              cout << "Zero points " << scene_pts_.block<3, 1>(0, j).transpose() << " slipping in at bdyidx " << body_idx[j] << endl;
            }
            avg_dist += (scene_pts_.block<3, 1>(0, j) - x.block<3, 1>(0, j)).norm();
            k++;
          }
        }
        if (k == 0) continue;
        avg_dist /= (double) k;

        // And now collect points that are closer than our allowance proportion * that avg dist
        k = 0;
        for (int j=0; j < body_idx.size(); j++){
          assert(k < body_idx.size());
          if (body_idx[j] == i){
            assert(j < scene_pts_.cols());
            if (scene_pts_(0, j) == 0.0){
              cout << "Zero points " << scene_pts_.block<3, 1>(0, j).transpose() << " slipping in at bdyidx " << body_idx[j] << endl;
            }
            if (optICPRejectionProp_ <= 0 || (scene_pts_.block<3, 1>(0, j) - x.block<3, 1>(0, j)).norm() <= avg_dist * optICPRejectionProp_){
              z.block<3, 1>(0, k) = scene_pts_.block<3, 1>(0, j);
              z_prime.block<3, 1>(0, k) = x.block<3, 1>(0, j);
              body_z_prime.block<3, 1>(0, k) = body_x.block<3, 1>(0, j);
              z_norms.block<3, 1>(0, k) = normal.block<3, 1>(0, j);
              k++;
            }
          }
        }
        z.conservativeResize(3, k);
        z_prime.conservativeResize(3, k);
        body_z_prime.conservativeResize(3, k);
        z_norms.conservativeResize(3, k);

        // forwardkin to get our jacobians at the project points on the body
        auto J = robot_.transformPointsJacobian(robot_kinematics_cache, body_z_prime, i, 0, false);

        // TODO(gizatt) Make point-to-plane work?
        bool POINT_TO_PLANE = true;
        for (int j=0; j < k; j++){
          MatrixXd Ks = z.col(j) - z_prime.col(j) + J.block(3*j, 0, 3, nq)*q_robot;
          if (POINT_TO_PLANE){
            //cout << z_norms.col(j).transpose() << endl;
            //cout << "Together: " << (z_norms.col(j) * z_norms.col(j).transpose()) << endl;
            f.block(0, 0, nq, 1) -= (2. * (z_norms.col(j).transpose() * Ks).transpose() * (z_norms.col(j).transpose() * J.block(3*j, 0, 3, nq))).transpose() / (double) k;
            Q.block(0, 0, nq, nq) += (2. *  J.block(3*j, 0, 3, nq).transpose() * z_norms.col(j) * z_norms.col(j).transpose() * J.block(3*j, 0, 3, nq)) / (double) k;
          } else {
            f.block(0, 0, nq, 1) -= (2. * Ks.transpose() * J.block(3*j, 0, 3, nq)).transpose() / (double) k;
            Q.block(0, 0, nq, nq) += (2. *  J.block(3*j, 0, 3, nq).transpose() * J.block(3*j, 0, 3, nq)) / (double) k;
          }
          K += Ks.squaredNorm() / (double) k;
        }
      }
    }

    if (fabs(K) > 0.0){
      // cut out variables that do not enter at all -- i.e., their row and column of Q, and row of f, are 0
      MatrixXd Q_reduced;
      VectorXd f_reduced;

      // is it used?
      std::vector<bool> rows_used(nq, false);
      int nq_reduced = 0;
      for (int i=0; i < nq; i++){
        if ( !(fabs(f[i]) <= 1E-10 && Q.block(i, 0, 1, nq).norm() <= 1E-10 && Q.block(0, i, nq, 1).norm() <= 1E-10) ){
          rows_used[i] = true;
          nq_reduced++;
        }
      }
      // do this reduction (collapse the rows/cols of vars that don't correspond)
      Q_reduced.resize(nq_reduced, nq_reduced);
      f_reduced.resize(nq_reduced, 1);
      int ir = 0, jr = 0;
      for (int i=0; i < nq; i++){
        if (rows_used[i]){
          jr = 0;
          for (int j=0; j < nq; j++){
            if (rows_used[j]){
              Q_reduced(ir, jr) = Q(i, j);
              jr++;
            }
          }
          f_reduced[ir] = f[i];
          ir++;
        }
      }

      // perform reduced solve
      ColPivHouseholderQR<MatrixXd> QR = Q_reduced.colPivHouseholderQr();
      VectorXd q_new_reduced = QR.solve(-f_reduced);
      MatrixXd Q_reduced_inverse = Q_reduced.inverse();

      // Detect bad solutions
      if (!is_finite(q_new_reduced)){
        break;
      }

      // Re-expand and accept this solution
      ir = 0;
      for (int i=0; i < nq; i++){
        if (rows_used[i] && q_new_reduced[ir] == q_new_reduced[ir]){
          // update of mean:
          q_robot[i] = q_new_reduced[ir];
          ir++;
        }
      }
    }

    if (getUnixTime() - last_published_icp > 0.01 && q_robot.transpose() * q_robot < 100.0){
      rm.publishRigidBodyTree(robot_, q_robot, Vector4d(0.5, 0.2, 0.2, 0.5), {"icp"});
      last_published_icp = getUnixTime();
    }
    //cout << "Error " << last_error << ", q " << q_robot << endl;
    usleep(1000*1);
  }

  // Get final objective
  double new_objective = 0.0;
  // Search closest points
  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot);
  VectorXd phi;
  Matrix3Xd normal;
  Matrix3Xd x;
  Matrix3Xd body_x;
  vector<int> body_idx;
  robot_.collisionDetectFromPoints(robot_kinematics_cache, scene_pts_, phi,
              normal, x, body_x, body_idx, false);

  for (int i = 0; i < phi.size(); i++){
    new_objective += fmin( optPhiMax_, (x.col(i) - scene_pts_.col(i)).lpNorm<1>());
  }

  if (q_robot.transpose() * q_robot < 100.0)
    rm.publishRigidBodyTree(robot_, q_robot, Vector4d(0.5, 0.2, 0.2, 0.5), {"icp"});
  printf("Testing objective %f\n", new_objective);
  if (new_objective < best_heuristic_supplied_yet_*1.1){
    printf("New near-best objective %f\n", new_objective);
    best_heuristic_supplied_yet_ = new_objective;
    NewHeuristicSol n;
    getInitialGuessFromRobotState(q_robot, n.vals, n.vars);
    new_heuristic_sols_lock_.lock();
    new_heuristic_sols_.push_back(n);
    new_heuristic_sols_lock_.unlock();
    printf("Heuristic queue len is now %ld, seed len is %ld\n", new_heuristic_sols_.size(), icp_search_seeds_.size());
  }
}

MIQPMultipleMeshModelDetector::MIQPMultipleMeshModelDetector(YAML::Node config){
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
  if (config["downsample_to_this_many_points"])
    optDownsampleToThisManyPoints_ = config["downsample_to_this_many_points"].as<int>();
  if (config["model_sample_rays"])
    optModelSampleRays_ = config["model_sample_rays"].as<int>();
  if (config["add_this_many_outliers"])
    optNumOutliers_ = config["add_this_many_outliers"].as<int>();
  if (config["scene_point_additive_noise"])
    optAddedSceneNoise_ = config["scene_point_additive_noise"].as<double>();
  if (config["big_M"])
    optBigNumber_ = config["big_M"].as<double>();
  if (config["ICP_prior_weight"])
     optICPPriorWeight_ = config["ICP_prior_weight"].as<double>();
  if (config["ICP_max_iters"])
    optICPIters_ = config["ICP_max_iters"].as<int>();
  if (config["ICP_outlier_rejection_proportion"])
    optICPRejectionProp_ = config["ICP_outlier_rejection_proportion"].as<double>();
  if (config["ICP_use_as_heuristic"])
    optUseICPHeuristic_ = config["ICP_use_as_heuristic"].as<bool>();


  if (config["model_point_rand_seed"])
    optModelPointRandSeed_ = config["model_point_rand_seed"].as<int>();
  if (config["scene_point_rand_seed"])
    optScenePointRandSeed_ = config["scene_point_rand_seed"].as<int>();
  if (config["init_guess_rand_seed"])
    optInitGuessRandSeed_ = config["init_guess_rand_seed"].as<int>();

  
  config_ = config;

  // Load the model itself
  if (config["models"] == NULL){
    runtime_error("Must specify models for object detector to work with.");
  }
  // Model will be a RigidBodyTree.
  q_robot_gt_.resize(0);
  int old_q_robot_gt_size = 0;
  for (auto iter=config["models"].begin(); iter!=config["models"].end(); iter++) {
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

void MIQPMultipleMeshModelDetector::doScenePointPreprocessing(const Eigen::Matrix3Xd& scene_pts_in, Eigen::Matrix3Xd& scene_pts_out){
  if (optDownsampleToThisManyPoints_ < 0) {
    scene_pts_out = scene_pts_in;
  } else {
    scene_pts_out.resize(3, optDownsampleToThisManyPoints_);
    VectorXi indices = VectorXi::LinSpaced(scene_pts_in.cols(), 0, scene_pts_in.cols());

    if (optScenePointRandSeed_ < 0)
      srand(time(NULL));
    else
      srand(optScenePointRandSeed_);
    std::random_shuffle(indices.data(), indices.data() + scene_pts_in.cols());
    for (int i = 0; i < optDownsampleToThisManyPoints_; i++) {
      scene_pts_out.col(i) = scene_pts_in.col(indices[i]);
    }    
  }
  // And then, on top of that, completely corrupt some points to random noise 
  // on [-1, 1]
  if (optNumOutliers_ > 0){
    if (optScenePointRandSeed_ < 0)
      srand(time(NULL));
    else
      srand(optScenePointRandSeed_);
    VectorXi indices = VectorXi::LinSpaced(scene_pts_in.cols(), 0, scene_pts_in.cols());
    std::random_shuffle(indices.data(), indices.data() + scene_pts_out.cols());
    for (int i = 0; i < optNumOutliers_; i++) { 
      scene_pts_out.col(i) = VectorXd::Random(3);
    }
  }
  if (optAddedSceneNoise_ > 0.0){
    std::default_random_engine generator(0);
    std::normal_distribution<double> distribution(0.0,optAddedSceneNoise_);
    for (int i = 0; i < scene_pts_out.cols(); i++) {
      for (int k = 0; k < 3; k++) {
        scene_pts_out(k, i) += distribution(generator);
      }
    }
  }

  RemoteTreeViewerWrapper rm;
  rm.publishPointCloud(scene_pts_out, {"scene_pts_downsampled"}, {0.1, 1.0, 0.1});
}

void MIQPMultipleMeshModelDetector::doModelPointSampling(Matrix3Xd& pts, MatrixXd& B){
  // Extract vertices and meshes from the RBT.
  Matrix3Xd all_vertices;
  DrakeShapes::TrianglesVector all_faces;
  vector<int> face_body_map; // Maps from a face index (into all_faces) to an RBT body index.
  collectBodyMeshesFromRBT(all_vertices, all_faces, face_body_map);

  // Collect the area of each face
  vector<double> face_cumulative_area = {0.0};

  for (const auto& face : all_faces){
    Vector3d a = all_vertices.col( face[0] );
    Vector3d b = all_vertices.col( face[1] );
    Vector3d c = all_vertices.col( face[2] );
    double area = ((b - a).cross(c - a)).norm() / 2.;
    face_cumulative_area.push_back(face_cumulative_area[face_cumulative_area.size()-1] + area);
  }

  // Normalize cumulative areas.
  for (int i=0; i<face_cumulative_area.size(); i++){
    face_cumulative_area[i] /= face_cumulative_area[face_cumulative_area.size() - 1];
  }

  // Always do this the same way.
  if (optModelPointRandSeed_ < 0)
    srand(time(NULL));
  else
    srand(optModelPointRandSeed_);
  pts.resize(3, optModelSampleRays_);
  B.resize(robot_.get_num_bodies()-1, optModelSampleRays_);
  B.setZero();
  int i = 0;
  while (i < optModelSampleRays_){
    // Pick the face we'll sample from
    double sample = randrange(1E-12, 1.0 - 1E-12);
    int k = 0;
    for (k=0; k<face_cumulative_area.size(); k++){
      if (face_cumulative_area[k] >= sample){
        break;
      }
    }
    k -= 1;

    Vector3d a = all_vertices.col(all_faces[k][0]);
    Vector3d b = all_vertices.col(all_faces[k][1]);
    Vector3d c = all_vertices.col(all_faces[k][2]);

    double s1 = randrange(0.0, 1.0); 
    double s2 = randrange(0.0, 1.0);
    if (s1 + s2 <= 1.0){
      Vector3d pt = a + 
                    s1 * (b - a) +
                    s2 * (c - a);
      pts.col(i) = pt;
      B(face_body_map[k]-1, i) = 1;
      i++;
    }
  }

  RemoteTreeViewerWrapper rm;
  rm.publishPointCloud(pts, {"model_pts_sampled"}, {1.0, 0.0, 0.0});
}

void MIQPMultipleMeshModelDetector::collectBodyMeshesFromRBT(Eigen::Matrix3Xd& all_vertices, 
                              DrakeShapes::TrianglesVector& all_faces, 
                              std::vector<int>& face_body_map){
  // Collect faces from each body in our internally-stored RBT,
  // (except the world, which we skip by starting body_i at 1 instead
  // of 0).
  for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++) {
    const RigidBody<double>& body = robot_.get_body(body_i);
    // Extract collision geometry by searching through this body's
    // corresponding collision element ids.
    auto collision_elems = body.get_collision_element_ids();
    for (const auto& collision_elem : body.get_collision_element_ids()) {
      auto element = robot_.FindCollisionElement(collision_elem);
      if (element->hasGeometry()){
        const DrakeShapes::Geometry & geometry = element->getGeometry();

        if (geometry.hasFaces()){
          Matrix3Xd points;
          geometry.getPoints(points);
          // Transform these point into body frame from the 
          // geometry-centric frame.
          points = element->getLocalTransform() * points;

          // Expand all_vertices to contain these new points, and append them.
          all_vertices.conservativeResize(3, points.cols() + all_vertices.cols());
          all_vertices.block(0, all_vertices.cols() - points.cols(), 3, points.cols()) = points;

          // Append the face descriptions as well, being careful to offset the vertex
          // indices by the preceding number of vertices in all_vertices.
          DrakeShapes::TrianglesVector faces;
          geometry.getFaces(&faces);
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
}

std::vector<MIQPMultipleMeshModelDetector::TransformationVars> 
MIQPMultipleMeshModelDetector::addTransformationVarsAndConstraints(MathematicalProgram& prog, bool world_to_body_direction){
  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_gt_);
  transform_by_object_.clear();

  for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){
    TransformationVars new_tr;
    char name_postfix[100];
    sprintf(name_postfix, "_%s_%d", robot_.get_body(body_i).get_model_name().c_str(), body_i);

    // Spawn translations and bound them to large but finite values.
    new_tr.T = prog.NewContinuousVariables<3>(string("T")+string(name_postfix));
    prog.AddBoundingBoxConstraint(-optBigNumber_*VectorXd::Ones(3), optBigNumber_*VectorXd::Ones(3), new_tr.T);

    // Spawn indicator variables for translations that indicate negativity vs positivity
    auto T_pos_indicators = prog.NewBinaryVariables<3>(string("T_pos_")+string(name_postfix));
    auto T_pos = prog.NewContinuousVariables<3>(string("T_pos_C_")+string(name_postfix));
    auto T_neg_indicators = prog.NewBinaryVariables<3>(string("T_neg_")+string(name_postfix));
    auto T_neg = prog.NewContinuousVariables<3>(string("T_neg_C_")+string(name_postfix));
    for (int k=0; k<3; k++) {
      prog.AddLinearEqualityConstraint(T_neg_indicators[k] + T_pos_indicators[k], 1.0);
      prog.AddLinearConstraint(T_pos[k] <= T_pos_indicators[k] * optBigNumber_);
      prog.AddLinearConstraint(T_neg[k] <= T_neg_indicators[k] * optBigNumber_);
      prog.AddLinearEqualityConstraint(T_pos[k] - T_neg[k] == new_tr.T[k]);
    }



    // Spawn rotations and constraint them in according to the configuration.
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
          new_tr.R_indicators = AddRotationMatrixMcCormickEnvelopeMilpConstraints(&prog, new_tr.R, optRotationConstraintNumFaces_);
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
      if (world_to_body_direction)
        ground_truth_tf = ground_truth_tf.inverse();
      // Formulas only work for vectors, so implement this constraint column-wise
      for (int i = 0; i < 3; ++i) {
        prog.AddLinearEqualityConstraint(new_tr.R.col(i) - ground_truth_tf.rotation().col(i), Eigen::Vector3d::Zero());
      }
    }
    transform_by_object_.push_back(new_tr);
  }
  return transform_by_object_;
}

double EnvelopeMinValue(int i, int num_binary_variables_per_half_axis) {
  return static_cast<double>(i) / num_binary_variables_per_half_axis;
}
void MIQPMultipleMeshModelDetector::getInitialGuessFromRobotState(const VectorXd& q_robot,
  VectorXd& vals, VectorXDecisionVariable& vars){
  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot);
    
  for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){
    Affine3d tf;
    
    if (config_["detector_type"].as<string>() == "body_to_world_transforms")
      tf = robot_.relativeTransform(robot_kinematics_cache, 0, body_i);
    else
      tf = robot_.relativeTransform(robot_kinematics_cache, body_i, 0);

    vars.conservativeResize(vars.size() + 12);
    vals.conservativeResize(vals.size() + 12);
    vals.block<3, 1>(vals.size() - 12, 0) = tf.translation();
    vars.block<3, 1>(vals.size() - 12, 0) = transform_by_object_[body_i - 1].T;
    for (int k = 0; k < 3; k++){
      vals.block<3, 1>(vals.size() - 9 + k*3 , 0) = tf.rotation().row(k);
      vars.block<3, 1>(vals.size() - 9 + k*3, 0) = transform_by_object_[body_i - 1].R.row(k);
    }

    // Using that rotation, set the indicator variables appropriately, if present

    //  forall k>=0, 0<=phi(k), and
    //  forall k<=num_binary_vars_per_half_axis, phi(k)<=1.

    //   Bpos[k](i,j) = 1 <=> R(i,j) >= phi(k)
    //   Bneg[k](i,j) = 1 <=> R(i,j) <= -phi(k)

    if (optRotationConstraint_ == 4){
      auto Bpos = std::get<2>(transform_by_object_[body_i - 1].R_indicators);
      auto Bneg = std::get<3>(transform_by_object_[body_i - 1].R_indicators);

      int K = Bpos.size();
      int offset = vals.size();
      vals.conservativeResize(offset + K*9*2);
      vars.conservativeResize(offset + K*9*2);
      for (int k = 0; k < K; k++ ){
        Matrix3d assigns_pos; assigns_pos.setZero();
        Matrix3d assigns_neg; assigns_neg.setZero();
        for (int x = 0; x < 3; x++){
          for (int y = 0; y < 3; y++){ 
            if (tf.rotation()(x, y) > EnvelopeMinValue(k, K)){
              assigns_pos(x, y) = 1.0;
            }
            if (tf.rotation()(x, y) < -EnvelopeMinValue(k, K)){
              assigns_neg(x, y) = 1.0;
            }
          }
        }
        if (k == 0){
          // For the zero-case, need to ensure we've assigned
          // the row and column vectors to different and non-opposite
          // orthants from each other.
          Matrix3d assigns = assigns_pos - assigns_neg;
          // Proceed rowwise...
          for (int row = 0; row < 3; row++){
            // (calculate number of possible assignments)        
            int num_zeros = 0; 
            for (int i = 0; i < 3; i++) 
              num_zeros += (assigns(row, i) == 0);
            if (num_zeros == 0) continue;

            // Try all assignments for the zeros of this row.
            for (int assignment = 0; assignment < 1<<num_zeros; assignment++){
              // Generate the assignment.
              Matrix3d trial_assignment; trial_assignment.setZero();
              int j = 0;
              for (int i = 0; i < 3; i++){
                if (assigns(row, i) == 0){
                  trial_assignment(row, i) = ((assignment >> j) & 1)*2-1;
                  j++;
                }
              }
              Matrix3d assigns_topNrows; assigns_topNrows.setZero();
              assigns_topNrows.block(0, 0, row+1, 3) = (assigns + trial_assignment).block(0, 0, row+1, 3);
              if (Eigen::FullPivLU<Matrix3d>(assigns_topNrows).rank() >= row+1){
                // full row rank so far, so take this assignment and proceed
                assigns += trial_assignment;
                break;
              }
            }  
          }
          // Back out assigns_pos and assigns_neg again
          for (int i = 0; i < 9; i++){
            if (assigns(i) > 0)
              assigns_pos(i) = 1.0;
            else if (assigns(i) < 0)
              assigns_neg(i) = 1.0;
          }
        }
        
        for (int x = 0; x < 3; x++){
          for (int y = 0; y < 3; y++){
            vars[offset] = Bpos[k](x, y);
            vals[offset] = assigns_pos(x, y);
            offset++;
            vars[offset] = Bneg[k](x, y);
            vals[offset] = assigns_neg(x, y);
            offset++;
          }
        }
      }
    }
  }

  if (config_["detector_type"].as<string>() == "world_to_body_transforms"){
    // for every scene point, project it down onto the models at the supplied TF to get closest object, and use 
    // that face assignment / point assignment as our guess if the distance isn't too great
    VectorXd search_phi;
    Matrix3Xd search_norm;
    Matrix3Xd search_x;
    Matrix3Xd search_body_x;
    vector<int> search_body_idx;
    robot_.collisionDetectFromPoints(robot_kinematics_cache, scene_pts_, search_phi,
                search_norm, search_x, search_body_x, search_body_idx, false);

    MatrixXd f0(scene_pts_.cols(), F_.rows());
    MatrixXd f_outlier0(scene_pts_.cols(), 1);
    f0.setZero();
    f_outlier0.setZero();
    //printf("Starting to backsolve initial guess...\n");
    // Prepare transforms for all bodies
    vector<Affine3d> tfs(robot_.get_num_bodies());
    for (int i=0; i < robot_.get_num_bodies(); i++){
      tfs[i] = robot_.relativeTransform(robot_kinematics_cache, 0, i+1);
    }
    for (int i=0; i<scene_pts_.cols(); i++){
      // Find the face it's closest to on body i
      double dist = std::numeric_limits<double>::infinity();
      int face_ind = 0;
      Vector3d closest_pt;
      if (search_body_idx[i] > 0){
        for (int j=0; j<all_faces_.size(); j++){
          if (B_(search_body_idx[i]-1, j) < 0.5) continue;

          vector<Vector3d> verts;
          for (int k=0; k<3; k++){
            verts.push_back(tfs[search_body_idx[i]-1] * all_vertices_.col(all_faces_[j](k)));
          }
          Vector3d new_closest_pt = closesPointOnTriangle(verts, scene_pts_.col(i));
          double new_dist = (new_closest_pt - scene_pts_.col(i)).lpNorm<1>();
          if (new_dist < dist){
            dist = new_dist;
            face_ind = j;
            closest_pt = new_closest_pt;
          }
        }
      }
      if (!optAllowOutliers_ || dist < optPhiMax_){
        f0(i, face_ind) = 1;
      } else {
        // else it's an outlier point
        f_outlier0(i) = 1;
      }
      vals.conservativeResize(vals.size() + f0.cols());
      vals.block(vals.size() - f0.cols(), 0, f0.cols(), 1) = f0.row(i);
      vars.conservativeResize(vars.size() + f0.cols());
      vars.block(vars.size() - f0.cols(), 0, f0.cols(), 1) = f_.row(i);
    }

    vals.conservativeResize(vals.size() + f_outlier0.cols());
    vals.block(vals.size() - f_outlier0.rows(), 0, f_outlier0.rows(), 1) = f_outlier0;
    vars.conservativeResize(vars.size() + f_outlier0.cols());
    vars.block(vars.size() - f_outlier0.rows(), 0, f_outlier0.rows(), 1) = f_outlier_;

  } else if (config_["detector_type"].as<string>() == "world_to_body_transforms_with_sampled_model_points"){
    // For every scene point, find closest model point, in terms of l1 norm. Use that.
    MatrixXd C0(C_.rows(), C_.cols());
    C0.setZero();
    for (int i = 0; i < scene_pts_.cols(); i++){
      int closest_model_pt = -1;
      double closest_distance = std::numeric_limits<double>::infinity();
      for (int j = 0; j < all_vertices_.cols(); j++){
        double l1_dist = (scene_pts_.col(i) - all_vertices_.col(j)).lpNorm<1>();
        if (l1_dist < closest_distance){
          closest_model_pt = j;
          closest_distance = l1_dist;
        }
      }
      if (!optAllowOutliers_ || closest_distance < optPhiMax_){
        if (closest_model_pt > C0.cols() || closest_model_pt < 0)
          runtime_error("Failed to find closest model point.");
        C0(i, closest_model_pt) = 1;
      }
      vals.conservativeResize(vals.size() + C0.cols());
      vals.block(vals.size() - C0.cols(), 0, C0.cols(), 1) = C0.row(i);
      vars.conservativeResize(vars.size() + C0.cols());
      vars.block(vars.size() - C0.cols(), 0, C0.cols(), 1) = C_.row(i);
    }

  } else {
    runtime_error("Can't set initialization for this solver type yet.");
  }
}


/*****************************************************************************


        *******************************************************

               This is an elaborate separator so I can find this
              part of the file. Documentation goes here.

        *******************************************************



 ****************************************************************************/
std::vector<MIQPMultipleMeshModelDetector::Solution> MIQPMultipleMeshModelDetector::doObjectDetectionWithWorldToBodyFormulation(const Eigen::Matrix3Xd& scene_pts){
  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_gt_);

  // Extract vertices and meshes from the RBT.
  all_vertices_.resize(0, 0);
  all_faces_.clear();
  face_body_map_.clear(); // Maps from a face index (into all_faces) to an RBT body index.
  collectBodyMeshesFromRBT(all_vertices_, all_faces_, face_body_map_);
  scene_pts_ = scene_pts;

  // See https://www.sharelatex.com/project/5850590c38884b7c6f6aedd1
  // for details on problem formulation.
  MathematicalProgram prog;

  // Reform the extracted verticies and meshes into
  // matrices for our optimization.
  // F(i, j) is 1 iff vertex j is a member of face i, 0 otherwise.
  // B(i, j) is 1 iff face j is a member of body i, 0 otherwise.
  F_.resize(all_faces_.size(), all_vertices_.cols());
  // (Remember not include the "world" body, so take 1 away from
  //  robot_.get_num_bodies().)
  B_.resize(robot_.get_num_bodies() - 1, all_faces_.size());
  B_.setZero();
  F_.setZero();
  for (int i=0; i<all_faces_.size(); i++){
    // Generate sub-block of face selection matrix F
    F_(i, all_faces_[i][0]) = 1.;
    F_(i, all_faces_[i][1]) = 1.;
    F_(i, all_faces_[i][2]) = 1.;
    // Don't include the "world" body.
    B_(face_body_map_[i]-1, i) = 1.0;
  }

  auto transform_by_object_ = addTransformationVarsAndConstraints(prog, false);

  // Allocate slacks to choose minimum L-1 norm over objects
  phi_ = prog.NewContinuousVariables(scene_pts.cols(), 1, "phi");

  // And slacks to store term-wise absolute value terms for L-1 norm calculation
  alpha_ = prog.NewContinuousVariables(3, scene_pts.cols(), "alpha");

  // Each row is a set of affine coefficinets relating the scene point to a combination
  // of vertices on a single face of the model
  C_ = prog.NewContinuousVariables(scene_pts.cols(), all_vertices_.cols(), "C");

  // Binary variable selects which face is being corresponded to
  f_ = prog.NewBinaryVariables(scene_pts.cols(), F_.rows(),"f");
  // And outlier-correspondence variables
  f_outlier_ = prog.NewBinaryVariables(scene_pts.cols(), 1, "f_outlier");
  if (!optAllowOutliers_){
    prog.AddLinearEqualityConstraint(RowVectorXd::Ones(scene_pts.cols()), 0.0, f_outlier_);
  }


  // Optimization pushes on slacks to make them tight (make them do their job)
  // (and normalize by number of pts for MSE calculation)
  prog.AddLinearCost( (1.0 / (double)scene_pts.cols()) * VectorXd::Ones(scene_pts.cols()), phi_);
  // And penalizes outliers
  prog.AddLinearCost( (1.0 / (double)scene_pts.cols()) * VectorXd::Ones(scene_pts.cols()) * optPhiMax_, f_outlier_);

  // Constrain slacks nonnegative, to help the estimation of lower bound in relaxation  
  prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), phi_);
  for (int k=0; k<3; k++){
    prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), {alpha_.row(k).transpose()});
  }

  // Constrain each row of C, plus the outlier allowance, to sum to 1
  // to make them proper affine coefficients
  Eigen::MatrixXd C1 = Eigen::MatrixXd::Ones(1, C_.cols()+1);
  // sum(C_i) + f_outlier_(i) = 1
  for (size_t k=0; k<C_.rows(); k++){
    prog.AddLinearEqualityConstraint(C1, 1.0, {C_.row(k).transpose(), f_outlier_.row(k)});
  }

  // Constrain each row of f, plus the outlier allowance, to sum to 1
  Eigen::MatrixXd f1 = Eigen::MatrixXd::Ones(1, f_.cols() + 1);
  for (size_t k=0; k<f_.rows(); k++){
    prog.AddLinearEqualityConstraint(f1, 1, {f_.row(k).transpose(), f_outlier_.row(k)});
  }

  // Force all elems of C in [0, 1]
  for (int i=0; i<C_.rows(); i++){
    for (int j=0; j<C_.cols(); j++){
      prog.AddBoundingBoxConstraint(0.0, 1.0, C_(i, j));
    }
  }

  // Force elems of C to be zero unless their corresponding vertex is a member
  // of an active face
  // That is,
  //   C_{i, j} <= F_{:, j}^T * f_{i, :}^T
  // or reorganized
  // [0] <= [F_{:, j}^T -1] [f_{i, :}^T C_{i, j}]
  //         
  for (int i=0; i<C_.rows(); i++){
    for (int j=0; j<C_.cols(); j++){
      prog.AddLinearConstraint(C_(i,j) <= (F_.col(j).transpose() * f_.row(i).transpose())(0, 0));
    }
  }



  printf("Starting to add correspondence costs... ");
  for (int i=0; i<scene_pts.cols(); i++){
    // constrain L-1 distance slack based on correspondences
    // phi_i >= 1^T alpha_{i}
    prog.AddLinearConstraint(phi_(i, 0) >= (RowVector3d::Ones() * alpha_.col(i))(0, 0));

    // If we're allowing outliers, we need to constrain each phi_i to be bigger than
    // a penalty amount if no faces are selected
    if (optAllowOutliers_){
      // I believe Big-M is OK here, as opposed to convex hull reform,
      // as we wouldn't gain important tightness for the convex hull reform.
      // (phi_i >= 0 always, as constrained above, which is the tightest 
      // lower bound we can always enforce).
      // phi_i >= phi_max - (ones * f_i)*BIG
      prog.AddLinearConstraint(phi_(i, 0) >= optPhiMax_ - optBigNumber_*(RowVectorXd::Ones(f_.cols()) * f_.row(i).transpose())(0, 0));
    }

    for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){
      // Similar logic here -- solutions will always bind in a lower bound,
      // which we've constrained >= 0 above, and can't do any better than.
      // alpha_i >= +(R_l s_i + T - M C_{i, :}^T) - Big x (1 - B_l * f_i)
      // alpha_i >= -(R_l s_i + T - M C_{i, :}^T) - Big x (1 - B_l * f_i)
      // (alpha is a slack var to implement the absolute value)
      Matrix<Expression, 3, 1> l1ErrorPos =
          transform_by_object_[body_i-1].R * scene_pts.col(i) + transform_by_object_[body_i-1].T
             - all_vertices_ * C_.row(i).transpose();
      Matrix<Expression, 3, 1> selector = Vector3d::Ones() * (optBigNumber_ * (VectorXd::Ones(1) - B_.row(body_i - 1) * f_.row(i).transpose()));
      for (int k=0; k<3; k++){
        prog.AddLinearConstraint( alpha_(k, i) >= l1ErrorPos(k) - selector(k) );
        prog.AddLinearConstraint( alpha_(k, i) >= -l1ErrorPos(k) - selector(k) );
      }
    }
  }
  printf("\n");

  GurobiSolver gurobi_solver;
  MosekSolver mosek_solver;

  prog.SetSolverOption(SolverType::kGurobi, "OutputFlag", 1);
  prog.SetSolverOption(SolverType::kGurobi, "LogToConsole", 1);
  prog.SetSolverOption(SolverType::kGurobi, "LogFile", "loggg.gur");
  prog.SetSolverOption(SolverType::kGurobi, "DisplayInterval", 5);

  prog.SetSolverOption(SolverType::kMosek, "MSK_IPAR_LOG", 1);

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
  
  if (config_["mosek_int_options"]){
    for (auto iter = config_["mosek_int_options"].begin();
         iter != config_["mosek_int_options"].end();
         iter++){
      prog.SetSolverOption(SolverType::kMosek, iter->first.as<string>(), iter->second.as<int>());
    }
  }
  if (config_["mosek_float_options"]){
    for (auto iter = config_["mosek_float_options"].begin();
         iter != config_["mosek_float_options"].end();
         iter++){
      prog.SetSolverOption(SolverType::kMosek, iter->first.as<string>(), iter->second.as<float>());
    }
  }
  for (const auto& tf : transform_by_object_) {
    for (const auto& vars : std::get<2>(tf.R_indicators))
      mosek_solver.set_branch_priority({vars.col(0), vars.col(1), vars.col(2)}, 10);
    for (const auto& vars : std::get<3>(tf.R_indicators))
      mosek_solver.set_branch_priority({vars.col(0), vars.col(1), vars.col(2)}, 10);
  }

  if (optUseInitialGuess_){
    // Randomize this in particular.
    srand(time(NULL));

    // Corruption should be done from q_gt, then recover maximal coordinates from transform queries,
    // get mesh and do projection using get_closest_points in drake 
    VectorXd corruption_vec(q_robot_gt_.rows()); 
    for (int i=0; i<corruption_vec.rows(); i++){
      corruption_vec[i] = randrange(-optCorruption_, optCorruption_);
    }
    VectorXd q_robot_corrupt = q_robot_gt_ + corruption_vec;
    KinematicsCache<double> robot_kinematics_cache_corrupt = robot_.doKinematics(q_robot_corrupt);
    cout << "q robot corrupt " << q_robot_corrupt.transpose() << endl;

    VectorXd vals;
    VectorXDecisionVariable vars;
    getInitialGuessFromRobotState(q_robot_corrupt, vals, vars);
    prog.SetInitialGuess(vars, vals);
  }

  //  prog.SetSolverOption(SolverType::kGurobi, "Cutoff", 50.0);
  // isn't doing anything... not invoking this tool right?
  //  prog.SetSolverOption(SolverType::kGurobi, "TuneJobs", 8);
  //  prog.SetSolverOption(SolverType::kGurobi, "TuneResults", 3);
  //prog.SetSolverOption(SolverType::kGurobi, )

  gurobi_solver.addMIPNodeCallback(&mipNodeCallbackFunction, this);
  gurobi_solver.addMIPSolCallback(&mipSolCallbackFunction, this);
  std::thread t;
  if (optUseICPHeuristic_){
    done = 0;
    t = std::thread(callICPProcessingForever, this);
  }

  double start_time = getUnixTime();
  auto out = gurobi_solver.Solve(prog);
  string problem_string = "rigidtf";
  double elapsed = getUnixTime() - start_time;

  if (optUseICPHeuristic_){
    done = 1;
    t.join();  
  }
  
  //prog.PrintSolution();
  printf("Code %d, problem %s solved for %lu scene solved in: %f\n", out, problem_string.c_str(), scene_pts.cols(), elapsed);  


  std::vector<MIQPMultipleMeshModelDetector::Solution> solutions;

  // for (int sol_i = 0; sol_i < prog.get_num_solutions(); sol_i++) {
  for (int sol_i = 0; sol_i < 1; sol_i ++){
    printf("==================================================\n");
    printf("======================SOL %d ======================\n", sol_i);
    printf("==================================================\n");
    auto f_est= prog.GetSolution(f_); //, sol_i);
    auto C_est = prog.GetSolution(C_); //, sol_i);

    MIQPMultipleMeshModelDetector::Solution new_solution;

    for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++){
      const RigidBody<double>& body = robot_.get_body(body_i);

      ObjectDetection new_detection;
      new_detection.obj_ind = body_i;
  
      printf("************************************************\n");
      printf("Concerning model %d (%s):\n", body_i, body.get_name().c_str());
      printf("------------------------------------------------\n");
      Vector3d Tf = prog.GetSolution(transform_by_object_[body_i-1].T); //, sol_i);
      Matrix3d Rf = prog.GetSolution(transform_by_object_[body_i-1].R); //, sol_i);
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
          if (f_est(scene_i, face_i) > 0.5 && B_(body_i-1, face_i) > 0.5){
            PointCorrespondence new_corresp;
            new_corresp.scene_pt = scene_pts.col(scene_i);
            new_corresp.model_pt = new_detection.est_tf * scene_pts.col(scene_i);
            new_corresp.scene_ind = scene_i;
            new_corresp.face_ind = face_i;
            for (int k_v=0; k_v<all_vertices_.cols(); k_v++){
              if (C_est(scene_i, k_v) >= 0.0){
                new_corresp.model_verts.push_back( all_vertices_.col(k_v) );
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

    new_solution.objective = prog.GetOptimalCost();
    new_solution.lower_bound = prog.GetLowerBound();
    new_solution.solve_time = elapsed;
    solutions.push_back(new_solution);
  }
  return solutions;
}

/*****************************************************************************


        *******************************************************

               This is an elaborate separator so I can find this
              part of the file. Documentation goes here.

        *******************************************************



 ****************************************************************************/
std::vector<MIQPMultipleMeshModelDetector::Solution> MIQPMultipleMeshModelDetector::doObjectDetectionWithWorldToBodyFormulationSampledModelPoints(const Eigen::Matrix3Xd& scene_pts){
  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_gt_);

  scene_pts_ = scene_pts;

  // See https://www.sharelatex.com/project/5850590c38884b7c6f6aedd1
  // for details on problem formulation.
  MathematicalProgram prog;

  // Add decision variables for the transformations for each body,
  // and add their transform-relevant constraints.
  transform_by_object_ = addTransformationVarsAndConstraints(prog, false);

  // Sample points from surface of model
  doModelPointSampling(all_vertices_, B_);

  // Add selection variables corresponding scene points to model-sampled points 
  C_ = prog.NewBinaryVariables(scene_pts.cols(), all_vertices_.cols(), "C");
  // Constrain that every row (every scene point) must have at most one active member
  // (i.e. must correspond to at most model point)
  if (optAllowOutliers_){
    for (int i = 0; i < scene_pts.cols(); i++) {
      prog.AddLinearConstraint(
        VectorXd::Ones(scene_pts.cols()).transpose() * C_.row(i).transpose() <= VectorXd::Ones(1.0));
    }
  } else {
    for (int i = 0; i < scene_pts.cols(); i++) {
      prog.AddLinearEqualityConstraint(
        VectorXd::Ones(scene_pts.cols()).transpose() * C_.row(i).transpose(), VectorXd::Ones(1.0));
    }
  }

  // Allocate slacks to choose minimum L-1 norm over objects
  phi_ = prog.NewContinuousVariables(scene_pts.cols(), 1, "phi");
  // And slacks to store term-wise absolute value terms for L-1 norm calculation
  alpha_ = prog.NewContinuousVariables(3, scene_pts.cols(), "alpha");
  // Optimization pushes on slacks to make them tight (make them do their job)
  // (and normalize by number of pts for MSE calculation)
  prog.AddLinearCost( (1.0 / (double)scene_pts.cols()) * VectorXd::Ones(scene_pts.cols()), phi_);
  // Constrain slacks nonnegative, to help the estimation of lower bound in relaxation  
  prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), phi_);
  for (int k=0; k<3; k++){
    prog.AddBoundingBoxConstraint(0.0, std::numeric_limits<double>::infinity(), {alpha_.row(k).transpose()});
  }

  for (int i=0; i<scene_pts.cols(); i++){
    // constrain L-1 distance slack based on correspondences
    // phi_i >= 1^T alpha_{i}
    prog.AddLinearConstraint(phi_(i, 0) >= (RowVector3d::Ones() * alpha_.col(i))(0, 0));

    // If we're allowing outliers, we need to constrain each phi_i to be bigger than
    // a penalty amount if no faces are selected
    if (optAllowOutliers_){
      // I believe Big-M is OK here, as opposed to convex hull reform,
      // as we wouldn't gain important tightness for the convex hull reform.
      // (phi_i >= 0 always, as constrained above, which is the tightest 
      // lower bound we can always enforce).
      // phi_i >= phi_max - (ones * f_i)*BIG
      prog.AddLinearConstraint(phi_(i, 0) >= optPhiMax_ - optBigNumber_*(RowVectorXd::Ones(C_.cols()) * C_.row(i).transpose())(0, 0));
    }

    for (int body_i=1; body_i<robot_.get_num_bodies(); body_i++){    
      // Similar logic here -- solutions will always bind in a lower bound,
      // which we've constrained >= 0 above, and can't do any better than.
      // alpha_i >= +(R_l s_i + T - M C_{i, :}^T) - Big x (1 - B_l * C_i)
      // alpha_i >= -(R_l s_i + T - M C_{i, :}^T) - Big x (1 - B_l * C_i)
      // (alpha is a slack var to implement the absolute value)
      Matrix<Expression, 3, 1> l1ErrorPos =
          transform_by_object_[body_i-1].R * scene_pts.col(i) + transform_by_object_[body_i-1].T
             - all_vertices_ * C_.row(i).transpose();
      Matrix<Expression, 3, 1> selector = Vector3d::Ones() * (optBigNumber_ * (VectorXd::Ones(1) - B_.row(body_i - 1) * C_.row(i).transpose()));
      for (int k=0; k<3; k++){
        prog.AddLinearConstraint( alpha_(k, i) >= l1ErrorPos(k) - selector(k) );
        prog.AddLinearConstraint( alpha_(k, i) >= -l1ErrorPos(k) - selector(k) );
      }
    }
  }


/*
  // For every scene point, add a cost for its correspondence.
  for (int i = 0; i < scene_pts.cols(); i++) {
    // TODO(gizatt) Generalize for more than 1 object.
    // This generates a linear expression for the scene point transformation into model frame:
    Matrix<Expression, 3, 1> transformed_model_pt_expr = transform_by_object_[0].R * scene_pts.col(i) + transform_by_object_[0].T;
    Matrix<Expression, 3, 1> selected_model_pt_expr(3, 1);
    // This generates a linear expression selecting a scene point using our permutation matrix:
    // TODO(gizatt) This might be able to be made into a one-liner with careful use of a diag() call...
    selected_model_pt_expr.block<1, 1>(0, 0) = C_.row(i) * all_vertices_.row(0).transpose();
    selected_model_pt_expr.block<1, 1>(1, 0) = C_.row(i) * all_vertices_.row(1).transpose();
    selected_model_pt_expr.block<1, 1>(2, 0) = C_.row(i) * all_vertices_.row(2).transpose();

    auto full_cost_expr = (transformed_model_pt_expr - selected_model_pt_expr).transpose() * (transformed_model_pt_expr - selected_model_pt_expr);
    prog.AddQuadraticCost(full_cost_expr);
  }
*/

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
    // Randomize this in particular.
    if (optInitGuessRandSeed_ < 0)
      srand(time(NULL));
    else
      srand(optInitGuessRandSeed_);

    // Corruption should be done from q_gt, then recover maximal coordinates from transform queries,
    // get mesh and do projection using get_closest_points in drake 
    VectorXd corruption_vec(q_robot_gt_.rows()); 
    for (int i=0; i<corruption_vec.rows(); i++){
      corruption_vec[i] = randrange(-optCorruption_, optCorruption_);
    }
    VectorXd q_robot_corrupt = q_robot_gt_ + corruption_vec;
    KinematicsCache<double> robot_kinematics_cache_corrupt = robot_.doKinematics(q_robot_corrupt);
    cout << "q robot corrupt " << q_robot_corrupt.transpose() << endl;

    VectorXd vals;
    VectorXDecisionVariable vars;
    getInitialGuessFromRobotState(q_robot_corrupt, vals, vars);
    prog.SetInitialGuess(vars, vals);
  }

  gurobi_solver.addMIPNodeCallback(&mipNodeCallbackFunction, this);
  gurobi_solver.addMIPSolCallback(&mipSolCallbackFunction, this);
  std::thread t;
  if (optUseICPHeuristic_){
    done = 0;
    t = std::thread(callICPProcessingForever, this);
  }

  double start_time = getUnixTime();
  auto out = gurobi_solver.Solve(prog);
  string problem_string = "rigidtf";
  double elapsed = getUnixTime() - start_time;

  if (optUseICPHeuristic_){
    done = 1;
    t.join();  
  }

  //prog.PrintSolution();
  printf("Code %d, problem %s solved for %lu scene solved in: %f\n", out, problem_string.c_str(), scene_pts.cols(), elapsed);

  std::vector<MIQPMultipleMeshModelDetector::Solution> solutions;

  for (int sol_i = 0; sol_i < 1; sol_i ++){
    printf("==================================================\n");
    printf("======================SOL %d ======================\n", sol_i);
    printf("==================================================\n");
    auto C_est = prog.GetSolution(C_); //, sol_i);

    MIQPMultipleMeshModelDetector::Solution new_solution;
    new_solution.objective = prog.GetOptimalCost();
    new_solution.lower_bound = prog.GetLowerBound();

    for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++){
      const RigidBody<double>& body = robot_.get_body(body_i);

      ObjectDetection new_detection;
      new_detection.obj_ind = body_i;
  
      printf("************************************************\n");
      printf("Concerning model %d (%s):\n", body_i, body.get_name().c_str());
      printf("------------------------------------------------\n");
      Vector3d Tf = prog.GetSolution(transform_by_object_[body_i-1].T); //, sol_i);
      Matrix3d Rf = prog.GetSolution(transform_by_object_[body_i-1].R); //, sol_i);
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
      new_detection.est_tf = new_detection.est_tf.inverse();
      
      for (int scene_i=0; scene_i<scene_pts.cols(); scene_i++) {
        for (int model_i=0; model_i<all_vertices_.cols(); model_i++) {
          if (C_est(scene_i, model_i) > 0.5) {
            PointCorrespondence new_corresp;
            new_corresp.scene_pt = scene_pts.col(scene_i);
            new_corresp.scene_ind = scene_i;
            new_corresp.model_pt = all_vertices_.col(model_i);
            new_detection.correspondences.push_back(new_corresp);
          }
        }
      }

      // Include this as a detection if we have correspondences
      // to support it.
      if (new_detection.correspondences.size() > 0)
        new_solution.detections.push_back(new_detection);
    }

    new_solution.solve_time = elapsed;
    solutions.push_back(new_solution);
  }

  return solutions;
}

/*****************************************************************************


        *******************************************************

               This is an elaborate separator so I can find this
              part of the file. Documentation goes here.

        *******************************************************



 ****************************************************************************/
std::vector<MIQPMultipleMeshModelDetector::Solution> MIQPMultipleMeshModelDetector::doObjectDetectionWithBodyToWorldFormulation(const Eigen::Matrix3Xd& scene_pts){
  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_gt_);

  // See https://www.sharelatex.com/project/5850590c38884b7c6f6aedd1
  // for details on problem formulation.
  MathematicalProgram prog;

  // Add decision variables for the transformations for each body,
  // and add their transform-relevant constraints.
  auto transform_by_object_ = addTransformationVarsAndConstraints(prog, true);

  // Sample points from surface of model
  Matrix3Xd model_pts;
  doModelPointSampling(model_pts, B_);

  // Add selection variables corresponding model-sampled points to
  auto C = prog.NewBinaryVariables(model_pts.cols(), scene_pts.cols(), "C");
  // Constrain that every row (every model point) must have one active member
  // (i.e. must correspond to one scene point)
  for (int i = 0; i < model_pts.cols(); i++) {
    prog.AddLinearEqualityConstraint(
      VectorXd::Ones(scene_pts.cols()).transpose() * C.row(i).transpose(), VectorXd::Ones(1.0));
  }

  // For every model point, add a cost for its correspondence.
  for (int i = 0; i < model_pts.cols(); i++) {
    // TODO(gizatt) Generalize for more than 1 object.
    // This generates a linear expression for the model point transformation into scene frame:
    Matrix<Expression, 3, 1> transformed_model_pt_expr = transform_by_object_[0].R * model_pts.col(i) + transform_by_object_[0].T;
    Matrix<Expression, 3, 1> selected_scene_pt_expr(3, 1);
    // This generates a linear expression selecting a scene point using our permutation matrix:
    // TODO(gizatt) This might be able to be made into a one-liner with careful use of a diag() call...
    selected_scene_pt_expr.block<1, 1>(0, 0) = C.row(i) * scene_pts.row(0).transpose();
    selected_scene_pt_expr.block<1, 1>(1, 0) = C.row(i) * scene_pts.row(1).transpose();
    selected_scene_pt_expr.block<1, 1>(2, 0) = C.row(i) * scene_pts.row(2).transpose();

    auto full_cost_expr = (transformed_model_pt_expr - selected_scene_pt_expr).transpose() * (transformed_model_pt_expr - selected_scene_pt_expr);
    prog.AddQuadraticCost(full_cost_expr);
  }

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

      auto e1_a = transform_by_object_[body_i-1].R * z_a + transform_by_object_[body_i-1].T;
      auto e1_b = transform_by_object_[body_i-1].R * z_b + transform_by_object_[body_i-1].T;
      auto e2_a = transform_by_object_[parent_id-1].R * z_a + transform_by_object_[parent_id-1].T;
      auto e2_b = transform_by_object_[parent_id-1].R * z_b + transform_by_object_[parent_id-1].T;

      prog.AddLinearEqualityConstraint(e1_a, p_a_1);
      prog.AddLinearEqualityConstraint(e1_b, p_b_1);
      prog.AddLinearEqualityConstraint(e2_a, p_a_2);
      prog.AddLinearEqualityConstraint(e2_b, p_b_2);
    }
  }
  */

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

  double start_time = getUnixTime();
  auto out = gurobi_solver.Solve(prog);
  string problem_string = "rigidtf";
  double elapsed = getUnixTime() - start_time;

  //prog.PrintSolution();
  printf("Code %d, problem %s solved for %lu scene solved in: %f\n", out, problem_string.c_str(), scene_pts.cols(), elapsed);

  std::vector<MIQPMultipleMeshModelDetector::Solution> solutions;

  for (int sol_i = 0; sol_i < 1; sol_i ++){
    printf("==================================================\n");
    printf("======================SOL %d ======================\n", sol_i);
    printf("==================================================\n");
    auto C_est = prog.GetSolution(C_); //, sol_i);

    MIQPMultipleMeshModelDetector::Solution new_solution;
    new_solution.objective = prog.GetOptimalCost();
    new_solution.lower_bound = prog.GetLowerBound();

    for (int body_i = 1; body_i < robot_.get_num_bodies(); body_i++){
      const RigidBody<double>& body = robot_.get_body(body_i);

      ObjectDetection new_detection;
      new_detection.obj_ind = body_i;
  
      printf("************************************************\n");
      printf("Concerning model %d (%s):\n", body_i, body.get_name().c_str());
      printf("------------------------------------------------\n");
      Vector3d Tf = prog.GetSolution(transform_by_object_[body_i-1].T); //, sol_i);
      Matrix3d Rf = prog.GetSolution(transform_by_object_[body_i-1].R); //, sol_i);
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
      
      for (int model_i=0; model_i<model_pts.cols(); model_i++) {
        for (int scene_i=0; scene_i<scene_pts.cols(); scene_i++) {
          if (C_est(model_i, scene_i) > 0.5) {
            PointCorrespondence new_corresp;
            new_corresp.scene_pt = scene_pts.col(scene_i);
            new_corresp.scene_ind = scene_i;
            new_corresp.model_pt = model_pts.col(model_i);
            new_detection.correspondences.push_back(new_corresp);
          }
        }
      }

      // Include this as a detection if we have correspondences
      // to support it.
      if (new_detection.correspondences.size() > 0)
        new_solution.detections.push_back(new_detection);
    }

    new_solution.solve_time = elapsed;
    solutions.push_back(new_solution);
  }

  return solutions;
}

std::vector<MIQPMultipleMeshModelDetector::Solution> MIQPMultipleMeshModelDetector::doObjectDetection(const Eigen::Matrix3Xd& scene_pts_in){
  Eigen::Matrix3Xd scene_pts;
  doScenePointPreprocessing(scene_pts_in, scene_pts);

  // Branch on formulation type.
  std::vector<MIQPMultipleMeshModelDetector::Solution> solutions;
  if (config_["detector_type"] == NULL){
    runtime_error("MIQPMultipleMeshModelDetector needs a detector type specified.");
  } else if (config_["detector_type"].as<string>() == 
             "world_to_body_transforms"){
    solutions = doObjectDetectionWithWorldToBodyFormulation(scene_pts);
  } else if (config_["detector_type"].as<string>() == 
            "world_to_body_transforms_with_sampled_model_points"){
    solutions = doObjectDetectionWithWorldToBodyFormulationSampledModelPoints(scene_pts);
  } else if (config_["detector_type"].as<string>() == 
             "body_to_world_transforms"){
    solutions = doObjectDetectionWithBodyToWorldFormulation(scene_pts);
  } else {
    runtime_error("MIQPMultipleMeshModelDetector detector type not understood.");
  }

  return solutions;
}