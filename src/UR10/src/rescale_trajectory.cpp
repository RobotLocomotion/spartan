#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "rescale_trajectory.hpp"
#include "drake/common/drake_path.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/eigen_types.h"
#include "yaml-cpp/yaml.h"

#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::GetDrakePath;

std::vector<Eigen::VectorXd> initQDot(std::vector<Eigen::VectorXd> q, double t_end){
    std::vector<Eigen::VectorXd> qdot={};
    
}

std::vector<double> scaleTime(RigidBodyTree<double>* model, std::vector<Eigen::VectorXd> t_sol, const char* constraintFileString){
    YAML::Node constraintFile = YAML::LoadFile(constraintFileString);
    if(constraintFile == NULL)
        std::runtime_error("Provide a valid config YAML file");

    //std::cout << (rescale_options["limits"][0].as<double>());
    std::vector<double> joint_limits ={};
    for (auto iter = constraintFile["limits"].begin(); iter != constraintFile["limits"].end(); iter++){
        double limit = (*iter).as<double>();
        joint_limits.push_back(limit);
    }

    int wrist1=3;
    std::cout << (model->actuators[0].effort_limit_max_) <<std::endl;
    VectorXd qstar = model->getZeroConfiguration();
    qstar(3) = 0.8;
    KinematicsCache<double> cache = model->doKinematics(qstar);
    Vector3d com0 = model->centerOfMass(cache);

  Vector3d wrist1_pt = Vector3d::Zero();
  Vector3d wrist1_pos0 = model->transformPoints(cache, wrist1_pt, wrist1, 0);

    int nT = 4;
    Vector2d tspan(0,1);
    double dt = tspan(1) / (nT-1);
    std::vector<double> t(nT,0);
    for (int i =0; i <nT; i++) {
        t[i] = dt * i;
    }
    MatrixXd q0 = qstar.replicate(1, nT);
    VectorXd qdot0 = VectorXd::Zero(model->get_num_velocities());

    Vector3d wrist1_pos_lb = wrist1_pos0;
    wrist1_pos_lb(0) += 0.1;
    wrist1_pos_lb(1) += 0.05;
    wrist1_pos_lb(2) += 0.25;
    Vector3d wrist1_pos_ub = wrist1_pos_lb;
    wrist1_pos_ub(2) += 0.25;
    Vector2d tspan_end;
    tspan_end << t[nT - 1], t[nT - 1];
    WorldPositionConstraint kc_wrist1(
            model, wrist1, wrist1_pt, wrist1_pos_lb, wrist1_pos_ub, tspan_end);

    std::vector<RigidBodyConstraint*> constraint_array;
    constraint_array.push_back(&kc_wrist1);

    IKoptions ikoptions(model);
    MatrixXd q_sol(model->get_num_positions(), nT);
    MatrixXd qdot_sol(model->get_num_velocities(), nT);
    MatrixXd qddot_sol(model->get_num_positions(), nT);
    int info = 0;
    std::vector<std::string> infeasible_constraint;

    inverseKinTraj(model, nT, t.data(), qdot0, q0, q0,
                  1, constraint_array.data(), ikoptions,
                     &q_sol, &qdot_sol, &qddot_sol, &info, &infeasible_constraint);
    std::cout << qddot_sol << std::endl;
    return joint_limits;
}

int main(){
    std::cout << "HI" << std::endl;
    RigidBodyTree<double> robot;
    drake::parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld("/home/local/ANT/clemsd/spartan/models/ur10/ur10_description/ur10_robot_half.urdf", &robot);
    Eigen::VectorXd v1(6);
    Eigen::VectorXd v2(6);
    Eigen::VectorXd v3(6);
    v1 << 0.0,0.0,0.0,0.0,0.0,0.0;
    v2 << 1.00,1.00,1.00,1.00,1.00,1.00;
    v3 << 2.00,2.00,2.00,2.00,2.00,2.00;
    std::vector<Eigen::VectorXd> simple_sol={v1,v2,v3};
    std::vector<double> scaled_time = scaleTime(&robot, simple_sol, "/home/local/ANT/clemsd/spartan/src/UR10/config/ur10Config.yaml");
    //std::cout << scaled_time[0] <<std::endl;
}
