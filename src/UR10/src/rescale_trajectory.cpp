#include "rescale_trajectory.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/common/eigen_types.h"
#include "yaml-cpp/yaml.h"

#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>

std::vector<double> scaleTime(RigidBodyTree<double>* model, std::vector<Eigen::VectorXd> q_sol, const char* constraintFileString){
    YAML::Node constraintFile = YAML::LoadFile(constraintFileString);
    if(constraintFile == NULL)
        std::runtime_error("Provide a valid config YAML file");

    //std::cout << (rescale_options["limits"][0].as<double>());
    std::vector<double> joint_limits ={};
    for (auto iter = constraintFile["limits"].begin(); iter != constraintFile["limits"].end(); iter++){
        double limit = (*iter).as<double>();
        joint_limits.push_back(limit);
    }
    for (auto qi = q_sol.begin(); qi != q_sol.end(); qi++){

    }
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
    v2 << 3.14,3.14,3.14,3.14,3.14,3.14;
    v3 << 9.42,9.42,9.42,9.42,6.28,6.28;
    std::vector<Eigen::VectorXd> simple_sol={v1,v2,v3};
    std::vector<double> scaled_time = scaleTime(&robot, simple_sol, "/home/local/ANT/clemsd/spartan/src/UR10/config/ur10Config.yaml");
    std::cout << scaled_time[0] <<std::endl;
}
