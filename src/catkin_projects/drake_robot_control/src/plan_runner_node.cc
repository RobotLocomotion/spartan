#include <common_utils/system_utils.h>
#include <gflags/gflags.h>
#include <drake_robot_control/plan_runner.h>

//ROS
#include "ros/ros.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "plan_runner");
  ros::NodeHandle nh("plan_runner"); // sets the node's namespace
  std::string config_filename;
  nh.getParam("param_filename", config_filename);
  auto runner =
      drake::robot_plan_runner::RobotPlanRunner::GetInstance(nh, config_filename);
  runner->Start();

  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
