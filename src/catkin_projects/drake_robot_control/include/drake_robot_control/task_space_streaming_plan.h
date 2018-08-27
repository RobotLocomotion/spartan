#pragma once

#include "ros/ros.h"
#include "robot_msgs/CartesianGoalPoint.h"

#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rigid_transform.h>
#include <drake_robot_control/trajectory_plan_base.h>

// ROS

namespace drake {
namespace robot_plan_runner {

class TaskSpaceStreamingPlan : public PlanBase {
public:
  TaskSpaceStreamingPlan(std::shared_ptr<const RigidBodyTreed> tree,
                         ros::NodeHandle &nh)
      : PlanBase(std::move(tree)),
        have_goal_(false) {
    setpoint_subscriber_ = std::make_shared<ros::Subscriber>(
      nh.subscribe(
        "/plan_runner/task_space_streaming_setpoint", 1,
        &TaskSpaceStreamingPlan::HandleSetpoint, this));
  }

  // Current robot state x = [q,v]
  // Current time t
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) override;

  void HandleSetpoint(const robot_msgs::CartesianGoalPoint::ConstPtr& msg);

 private:
    std::mutex goal_mutex_;
    Eigen::Vector3d xyz_ee_goal_;
    Eigen::Vector3d xyz_d_ee_goal_;
    Eigen::Quaterniond quat_ee_goal_;
    // This is the frame the above goal points are expressed in:
    int body_index_ee_goal_;
    // This is the frame on the robot that is attempted to be
    // aligned to the above pose:
    int body_index_ee_frame_;
    bool have_goal_;

    std::shared_ptr<ros::Subscriber> setpoint_subscriber_;

    drake::TwistMatrix<double> J_ee_E_;
    drake::TwistMatrix<double> J_ee_W_;
    Eigen::Isometry3d H_WE_; // ee to world, current homogeneous transform
    math::RigidTransform<double>
      H_WEr_; // end-effector to world, reference homogeneous transform
  
    Eigen::Vector3d kp_rotation_;
    Eigen::Vector3d kp_translation_;

    double last_control_update_t_ = 0.;
};

} // namespace robot_plan_runner
} // namespace drake