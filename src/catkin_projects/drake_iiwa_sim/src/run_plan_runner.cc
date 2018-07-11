#include <robot_plan_runner/plan_runner.h>

int main() {
  drake::robot_plan_runner::RobotPlanRunner runner;
  runner.Start();
  return 0;
}