#pragma once

#include <stdexcept>
#include <iostream>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/rotation_constraint.h"
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

template <typename Derived>    
static void addMcCormickQuaternionConstraint(MathematicalProgram& prog, 
                              const Derived R,
                              int M_x,
                              int M_y){
    // constrain rotations to SO(3) -- i.e.
    // R.' R = I, and det(R) = +1
    // Add core quaternion variables, ordered w x y z
    auto Q = prog.NewContinuousVariables(4, 1, "q");
    prog.AddBoundingBoxConstraint(-VectorXd::Ones(4), VectorXd::Ones(4), Q);

    // Add variables for bilinear quaternion element products
    auto B = prog.NewContinuousVariables(10, 1, "b");
    prog.AddBoundingBoxConstraint(-VectorXd::Ones(10), VectorXd::Ones(10), B);   

    // Constrain elements of rotation element by bilinear quaternion values
    // This constrains the 9 elements of the rotation matrix against the 
    // 10 bilinear terms in various combinations
    MatrixXd Aeq(9, 9 + 10);
    Aeq.setZero();
    MatrixXd beq(9, 1);
    beq.setZero();
    // build some utility inds to make writing this cleaner...
    int k=0;
    char qnames[5] = "wxyz";
    const int kNumRotVars = 9;
    const int kOffww = kNumRotVars + 0;
    const int kOffwx = kNumRotVars + 1;
    const int kOffwy = kNumRotVars + 2;
    const int kOffwz = kNumRotVars + 3;
    const int kOffxx = kNumRotVars + 4;
    const int kOffxy = kNumRotVars + 5;
    const int kOffxz = kNumRotVars + 6;
    const int kOffyy = kNumRotVars + 7;
    const int kOffyz = kNumRotVars + 8;
    const int kOffzz = kNumRotVars + 9;
    // Todo: I know you can do this formulaicaijasafcally...
    // R00 = w^2 + x^2 - y^2 - z^2
    Aeq(k, 0) = 1.0;
    Aeq(k, kOffww) = -1.0;
    Aeq(k, kOffxx) = -1.0;
    Aeq(k, kOffyy) = 1.0;
    Aeq(k, kOffzz) = 1.0;
    beq(k, 0) = 0.0;
    k++;
    // R01 = 2xy + 2wz -> R01 - 2xy - 2wz = 0
    Aeq(k, 3) = 1.0;
    Aeq(k, kOffxy) = -2.0;
    Aeq(k, kOffwz) = -2.0;
    beq(k, 0) = 0.0;
    k++;
    // R02 = 2xz - 2wy -> R02 - 2xz + 2wy = 0
    Aeq(k, 6) = 1.0;
    Aeq(k, kOffxz) = -2.0;
    Aeq(k, kOffwy) = 2.0;
    beq(k, 0) = 0.0;
    k++;
    // R10 = 2xy - 2wz -> R10 - 2xy + 2wz = 0
    Aeq(k, 1) = 1.0;
    Aeq(k, kOffxy) = -2;
    Aeq(k, kOffwz) = 2;
    beq(k, 0) = 0.0;
    k++;
    // R11 = w^2 - x^2 + y^2 - z^2
    Aeq(k, 4) = 1.0;
    Aeq(k, kOffww) = -1.0;
    Aeq(k, kOffxx) = 1.0;
    Aeq(k, kOffyy) = -1.0;
    Aeq(k, kOffzz) = 1.0;
    beq(k, 0) = 0.0;
    k++;
    // R12 = 2yz + 2wx -> r12 - 2yz - 2wx = 0
    Aeq(k, 7) = 1.0;
    Aeq(k, kOffyz) = -2.0;
    Aeq(k, kOffwx) = -2.0;
    beq(k, 0) = 0.0;
    k++;
    // R20 = 2xz + 2wy -> r20 - 2xz - 2wy = 0
    Aeq(k, 2) = 1.0;
    Aeq(k, kOffxz) = -2.0;
    Aeq(k, kOffwy) = -2.0;
    beq(k, 0) = 0.0;
    k++;
    // R21 = 2yz - 2wx -> r21 - 2yz + 2wx = 0
    Aeq(k, 5) = 1.0;
    Aeq(k, kOffyz) = -2.0;
    Aeq(k, kOffwx) = 2.0;
    beq(k, 0) = 0.0;
    k++;
    // R22 = w^2 - x^2 - y^2 + z^2
    Aeq(k, 8) = 1.0;
    Aeq(k, kOffww) = -1.0;
    Aeq(k, kOffxx) = 1.0;
    Aeq(k, kOffyy) = 1.0;
    Aeq(k, kOffzz) = -1.0;
    beq(k, 0) = 0.0;
    k++;
    prog.AddLinearEqualityConstraint(Aeq, beq, {flatten_MxN(R), B});
    // Now constrain xx + yy + zz + ww = 1
    prog.AddLinearEqualityConstraint(MatrixXd::Ones(1, 4), MatrixXd::Ones(1, 1), 
      {B.block<1,1>(0,0),B.block<1,1>(4,0),B.block<1,1>(7,0),B.block<1,1>(9,0)});
    // Finally, constrain each of the bilinear product pairs with their core quaternion variables
    k=0;
    for (int i=0; i<4; i++){
      for (int j=i; j<4; j++){
        // spawn new region selection variables
        string corename; corename += qnames[i]; corename += qnames[j];

        // select variable "x" and "y" out of quaternion
        auto x = Q(i, 0);
        auto y = Q(j, 0);
        // and select bilinear product "xy" variable
        auto xy = B(k,0);

        Add2DLogarithmicMcCormickEnvelope(&prog, xy, x, y, corename,
            -1.0, 1.0, -1.0, 1.0, M_x, M_y);
        /*
        add_McCormick_envelope(prog, xy, x, y, corename,
                               -1.0, // xL
                               1.0,  // xH
                               -1.0, // yL
                               1.0,  // yH
                               M_x, M_y); // M_x, M_y 
        */
        k++;
      }
    }
}