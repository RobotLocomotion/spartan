#pragma once
#include <drake_robot_control/trajectory_plan_base.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// ROS

namespace drake {
namespace robot_plan_runner {

class JointSpaceStreamingPlan : public PlanBase {
public:
  JointSpaceStreamingPlan(std::shared_ptr<const RigidBodyTreed> tree,
                          ros::NodeHandle &nh)
      : PlanBase(std::move(tree)) {
    setpoint_subscriber_ = std::make_shared<ros::Subscriber>(
      nh.subscribe(
        "/plan_runner/joint_space_streaming_setpoint", 1,
        &JointSpaceStreamingPlan::HandleSetpoint, this));
  }

  // Current robot state x = [q,v]
  // Current time t
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) override;

  inline void SetGoal(const Eigen::VectorXd& q_commanded,
                      const Eigen::VectorXd& v_commanded,
                      const Eigen::VectorXd& tau_commanded) {
    DRAKE_ASSERT(q_commanded.rows() == get_num_positions());
    DRAKE_ASSERT(v_commanded.rows() == get_num_velocities());
    DRAKE_ASSERT(tau_commanded.rows() == get_num_positions());
    std::lock_guard<std::mutex> lock(goal_mutex_);
    q_commanded_ = q_commanded;
    v_commanded_ = v_commanded;
    tau_commanded_ = tau_commanded;
  }

  void HandleSetpoint(const sensor_msgs::JointState::ConstPtr& msg);

 private:
    Eigen::VectorXd q_commanded_;
    Eigen::VectorXd v_commanded_;
    Eigen::VectorXd tau_commanded_;
    std::mutex goal_mutex_;

    std::shared_ptr<ros::Subscriber> setpoint_subscriber_;
};

} // namespace robot_plan_runner
} // namespace drake