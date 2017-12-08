#include "iterative_closest_point.hpp"

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_type.h"

#include "common_utils/math_utils.h"

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::solvers::MathematicalProgram;
using drake::symbolic::Expression;

VectorXd compute_articulated_icp_update_for_points(
    RigidBodyTree<double>& robot, const VectorXd& q_robot,
    const Matrix3Xd& points, double prior_weight,
    double outlier_rejection_portion, bool use_point_to_plane) {
  int nq = robot.get_num_positions();
  if (q_robot.rows() != nq) {
    printf("Got %ld positions in q_robot but need %d from the RBT.\n",
           q_robot.rows(), nq);
    exit(-1);
  }
  KinematicsCache<double> robot_kinematics_cache = robot.doKinematics(q_robot);

  MathematicalProgram prog;
  auto q_update = prog.NewContinuousVariables(nq, "q");

  // Prior for ICP steps will be the last pose
  prog.AddQuadraticErrorCost(MatrixXd::Identity(nq, nq) * prior_weight, q_robot,
                             q_update);

  // Do closest point search to find our instantaneous correspondences
  VectorXd phi;
  Matrix3Xd normal;
  Matrix3Xd x;
  Matrix3Xd body_x;
  std::vector<int> body_idx;
  robot.collisionDetectFromPoints(robot_kinematics_cache, points, phi, normal,
                                  x, body_x, body_idx, false);

  // Prepare to group per body, since we'll get major performance gains
  // by doing bulk forward kinematic transforms on many points corresponding
  // to a single body at once.
  std::vector<int> num_points_on_body(robot.get_num_bodies(), 0);
  for (int i = 0; i < body_idx.size(); i++) {
    if (body_idx[i] >= 0) num_points_on_body[body_idx[i]] += 1;
  }

  // For every body...
  for (int i = 0; i < robot.get_num_bodies(); i++) {
    if (num_points_on_body[i] > 0) {
      // Collect results from closest-point lookup that correspond to this body.
      Matrix3Xd z(
          3, num_points_on_body[i]);  // points, in world frame, near this body
      Matrix3Xd z_prime(3, num_points_on_body[i]);  // same points projected
                                                    // onto surface of body
      Matrix3Xd body_z_prime(
          3, num_points_on_body[i]);  // projected points in body frame
      Matrix3Xd z_norms(
          3, num_points_on_body[i]);  // normals corresponding to these points
      int k = 0;

      // First collect average distance to nearby points
      double avg_dist = 0.0;
      for (int j = 0; j < body_idx.size(); j++) {
        assert(k < body_idx.size());
        if (body_idx[j] == i) {
          assert(j < points.cols());
          if (points(0, j) == 0.0) {
            std::cout << "Zero points " << points.block<3, 1>(0, j).transpose()
                      << " slipping in at bdyidx " << body_idx[j] << std::endl;
          }
          avg_dist += (points.block<3, 1>(0, j) - x.block<3, 1>(0, j)).norm();
          k++;
        }
      }
      if (k == 0) continue;
      avg_dist /= (double)k;

      // And now collect points that are closer than our allowance proportion
      // * that avg dist (i.e. this is where outlier rejection happens)
      k = 0;
      for (int j = 0; j < body_idx.size(); j++) {
        assert(k < body_idx.size());
        if (body_idx[j] == i) {
          assert(j < points.cols());
          if (points(0, j) == 0.0) {
            std::cout << "Zero points " << points.block<3, 1>(0, j).transpose()
                      << " slipping in at bdyidx " << body_idx[j] << std::endl;
          }
          if (outlier_rejection_portion <= 0 ||
              (points.block<3, 1>(0, j) - x.block<3, 1>(0, j)).norm() <=
                  avg_dist * outlier_rejection_portion) {
            z.block<3, 1>(0, k) = points.block<3, 1>(0, j);
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

      // forwardkin to get our jacobians at the projected points on the body
      auto J = robot.transformPointsJacobian(robot_kinematics_cache,
                                             body_z_prime, i, 0, false);

      // Create quadratic error cost, constructed by hand to 
      // avoid the large setup cost of Expressions...
      VectorXd f(nq);
      f.setZero();
      MatrixXd Q(nq, nq);
      Q.setZero();
      double K = 0.;
      for (int j = 0; j < k; j++) {
        MatrixXd Ks =
            z.col(j) - z_prime.col(j) + J.block(3 * j, 0, 3, nq) * q_robot;
        if (use_point_to_plane) {
          f.block(0, 0, nq, 1) -=
              (2. * (z_norms.col(j).transpose() * Ks).transpose() *
               (z_norms.col(j).transpose() * J.block(3 * j, 0, 3, nq)))
                  .transpose() /
              (double)k;
          Q.block(0, 0, nq, nq) +=
              (2. * J.block(3 * j, 0, 3, nq).transpose() * z_norms.col(j) *
               z_norms.col(j).transpose() * J.block(3 * j, 0, 3, nq)) /
              (double)k;
        } else {
          f.block(0, 0, nq, 1) -=
              (2. * Ks.transpose() * J.block(3 * j, 0, 3, nq)).transpose() /
              (double)k;
          Q.block(0, 0, nq, nq) +=
              (2. * J.block(3 * j, 0, 3, nq).transpose() *
               J.block(3 * j, 0, 3, nq)) /
              (double)k;
        }
        K += Ks.squaredNorm() / (double)k;
      }
      prog.AddQuadraticCost(Q, f, K, q_update);
    }
  }

  auto result = prog.Solve();
  return prog.GetSolution(q_update);
}
