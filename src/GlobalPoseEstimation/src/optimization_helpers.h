#pragma once

#include <stdexcept>
#include <iostream>

#include "drake/solvers/mathematical_program.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

#include <lcm/lcm-cpp.hpp>

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"
#include "unistd.h"

#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;
using namespace drake::solvers;

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

static double pointDistance(const PointType& pt1, const PointType& pt2){
  return sqrtf(powf(pt1.x - pt2.x, 2) + powf(pt1.y - pt2.y, 2) + powf(pt1.z - pt2.z, 2));
}

#include <time.h>
// call this function to start a nanosecond-resolution timer
static struct timespec timer_start(){
    struct timespec start_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
    return start_time;
}
// call this function to end a timer, returning nanoseconds elapsed as a long
static long timer_end(struct timespec start_time){
    struct timespec end_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);
    long diffInNanos = end_time.tv_nsec - start_time.tv_nsec;
    return diffInNanos;
}

// TODO: replace with reshape?
template <typename Derived, int rows, int cols>
static Eigen::Matrix<Derived, -1, -1> flatten_MxN( const Eigen::Matrix<Derived, rows, cols> & x ){
  Eigen::Matrix<Derived, -1, -1> ret(x.rows()*x.cols(), 1);
  for (int i=0; i<x.rows(); i++){ // for each row, paste that row, in order,
                                  // as elems in the new column vector
    ret.block(i*x.cols(), 0, x.cols(), 1) = x.block(i, 0, 1, x.cols()).transpose();
  }
  return ret;
}

static void add_McCormick_envelope(MathematicalProgram& prog, 
                              drake::symbolic::Variable& w,
                              drake::symbolic::Variable& x, 
                              drake::symbolic::Variable& y, 
                              string corename,
                              double xL, 
                              double xH, 
                              double yL, 
                              double yH, 
                              int M_x,
                              int M_y){
  MatrixXDecisionVariable x_mat(1,1); x_mat(0,0) = x;
  MatrixXDecisionVariable y_mat(1,1); y_mat(0,0) = y;
  MatrixXDecisionVariable w_mat(1,1); w_mat(0,0) = w;

  // Add binary variables for the region we are in
  const double kStepSizeX =  (xH - xL) / (double)M_x;
  const double kStepSizeY =  (yH - yL) / (double)M_y;

  auto z_uv = prog.NewBinaryVariables(M_x, M_y, (corename + "_z").c_str());
  // and constrain that we can be in one at a time
  prog.AddLinearEqualityConstraint(MatrixXd::Ones(1, M_x*M_y), MatrixXd::Ones(1, 1), flatten_MxN(z_uv));

  // Create indicator variables xhat and yhat for each subsection
  auto x_hat = prog.NewContinuousVariables(M_x, M_y, (corename + string("_xhat")).c_str());
  auto y_hat = prog.NewContinuousVariables(M_x, M_y, (corename + string("_yhat")).c_str());
  // They must sum to their respective variable...
  MatrixXd A_sumconstr = MatrixXd::Ones(1, 1+M_x*M_y);
  A_sumconstr(0, 0) = -1;
  prog.AddLinearEqualityConstraint(A_sumconstr, MatrixXd::Zero(1, 1), {x_mat, flatten_MxN(x_hat)});
  prog.AddLinearEqualityConstraint(A_sumconstr, MatrixXd::Zero(1, 1), {y_mat, flatten_MxN(y_hat)});
  // And respect the range of the region they represent -- which may force to zero if the region isn't active
  // Implemented as a bunch of upper and lower bounds
  MatrixXd A_region_bounds_xhat = MatrixXd::Zero(M_x*M_y*2, M_x*M_y + M_x*M_y);
  MatrixXd A_region_bounds_yhat = MatrixXd::Zero(M_x*M_y*2, M_x*M_y + M_x*M_y);
  MatrixXd lb_zero = MatrixXd::Zero(M_x*M_y*2, 1);
  MatrixXd ub_inf =  MatrixXd::Constant(M_x*M_y*2, 1, std::numeric_limits<double>::infinity());
  int k=0;
  for (int u=0; u<M_x; u++){
    for (int v=0; v<M_y; v++){
      double xL_uv = xL + u*kStepSizeX;
      double xH_uv = xL + (u+1)*kStepSizeX;
      double yL_uv = yL + v*kStepSizeY;
      double yH_uv = yL + (v+1)*kStepSizeY;
      // z(u,v) * xL(u,v) <= x_hat(u,v) <= z(u,v) * xH(u,v)
      A_region_bounds_xhat(2*k, k) = 1.0; // xhat - z(u,v) * xL(u,v) >= 0
      A_region_bounds_xhat(2*k, M_x*M_y+k) = -xL_uv;
      A_region_bounds_yhat(2*k, k) = 1.0; // yhat - z(u,v) * yL(u,v) >= 0
      A_region_bounds_yhat(2*k, M_x*M_y+k) = -yL_uv;

      A_region_bounds_xhat(2*k+1, k) = -1.0; // z(u,v) * xH(u,v) - xhat >= 0
      A_region_bounds_xhat(2*k+1, M_x*M_y+k) = xH_uv;
      A_region_bounds_yhat(2*k+1, k) = -1.0; // z(u,v) * yH(u,v) - yhat >= 0
      A_region_bounds_yhat(2*k+1, M_x*M_y+k) = yH_uv;
      k++;
    }
  }
  prog.AddLinearConstraint(A_region_bounds_xhat, lb_zero, ub_inf, {flatten_MxN(x_hat), flatten_MxN(z_uv)});
  prog.AddLinearConstraint(A_region_bounds_yhat, lb_zero, ub_inf, {flatten_MxN(y_hat), flatten_MxN(z_uv)});

  // And finally, constrain w by the four appropriate surfaces
  // Constraints w, xhats, yhats, z_uvs
  MatrixXd A_w_constraints = MatrixXd::Zero(4, 1 + M_x*M_y + M_x*M_y + M_x*M_y);
  const int xhat_s = 1;
  const int yhat_s = xhat_s + M_x*M_y;
  const int zuv_s = yhat_s + M_x*M_y;
  lb_zero = MatrixXd::Zero(4, 1);
  ub_inf = MatrixXd::Constant(4, 1, std::numeric_limits<double>::infinity());
  k=0;
  for (int u=0; u<M_x; u++){
    for (int v=0; v<M_y; v++){
      double xL_uv = xL + u*kStepSizeX;
      double xH_uv = xL + (u+1)*kStepSizeX;
      double yL_uv = yL + v*kStepSizeY;
      double yH_uv = yL + (v+1)*kStepSizeY;
      // w >= sum_{uv} xL(u,v) * y_hat(u,v) + x_hat(u,v) * yL(u,v) - xL(u,v)*yL(u,v)*z(u,v)
      A_w_constraints(0, 0) = 1.0;
      A_w_constraints(0, xhat_s + k) = - yL_uv;
      A_w_constraints(0, yhat_s + k) = - xL_uv;
      //lb_zero(0) = -xL_uv * yL_uv;
      A_w_constraints(0, zuv_s  + k) = xL_uv * yL_uv;

      // w >= sum_{uv} xH(u,v) * y_hat(u,v) + x_hat(u,v) * yH(u,v) - xH(u,v)*yH(u,v)*z(u,v)
      A_w_constraints(1, 0) = 1.0;
      A_w_constraints(1, xhat_s + k) = - yH_uv;
      A_w_constraints(1, yhat_s + k) = - xH_uv;
      //lb_zero(1) = -xH_uv * yH_uv;
      A_w_constraints(1, zuv_s  + k) = xH_uv * yH_uv;

      // w <= sum_{uv} xH(u,v) * y_hat(u,v) + x_hat(u,v) * yL(u,v) - xH(u,v)*yL(u,v)*z(u,v)
      A_w_constraints(2, 0) = -1.0;
      A_w_constraints(2, xhat_s + k) = yL_uv;
      A_w_constraints(2, yhat_s + k) = xH_uv;
      //lb_zero(2) = xH_uv * yL_uv;
      A_w_constraints(2, zuv_s  + k) = -xH_uv * yL_uv;

      // w <= sum_{uv} xH(u,v) * y_hat(u,v) + x_hat(u,v) * yL(u,v) - xH(u,v)*yL(u,v)*z(u,v)
      A_w_constraints(3, 0) = -1.0;
      A_w_constraints(3, xhat_s + k) = yH_uv;
      A_w_constraints(3, yhat_s + k) = xL_uv;
      //lb_zero(3) = xL_uv * yH_uv;
      A_w_constraints(3, zuv_s  + k) = -xL_uv * yH_uv;

      k++;
    }
  }
  prog.AddLinearConstraint(A_w_constraints, lb_zero, ub_inf, {w_mat, flatten_MxN(x_hat), flatten_MxN(y_hat), flatten_MxN(z_uv)});
}

