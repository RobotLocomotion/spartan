#include "rescale_trajectory.hpp"

std::vector<double> scaleTime(RigidBodyTree<double>* model, std::vector<Eigen::VectorXd> q_sol, const char* constraintFileString){
    YAML:Node constraintFile = YAML::LoadFile(constraintFileString);
    if(constraintFile == NULL)
        runtime_error("Provide a valid config YAML file");

    auto rescale_options=constraintFile["time_rescale_options"];
    if(rescale_options == NULL){
        runtime_error("Could not find time_rescale_options in yaml file");
    }

    std::vector<double> joint_limits ={};
    for (auto iter = rescale_options["limits"].begin(); iter != rescale_options["limits"].end(); iter++){
        double limit = (*iter).as<double>();
        joint_limits.push_back(limit);
    }
    return limits;
}
