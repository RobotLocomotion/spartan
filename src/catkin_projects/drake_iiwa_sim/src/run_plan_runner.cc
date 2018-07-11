#include <common_utils/system_utils.h>
#include <gflags/gflags.h>
#include <robot_plan_runner/plan_runner.h>

DEFINE_string(config_file_name, "${SPARTAN_SOURCE_DIR}/src/catkin_projects/"
                                "drake_iiwa_sim/config/"
                                "iiwa_plan_runner_config.yaml",
              "plan runner yaml config file path");

int main() {
  std::string config_file_name = FLAGS_config_file_name;
  autoExpandEnvironmentVariables(config_file_name);
  auto runner =
      drake::robot_plan_runner::RobotPlanRunner::GetInstance(config_file_name);
  runner->Start();
  return 0;
}