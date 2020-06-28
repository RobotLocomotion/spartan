#pragma once

// ROS
#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

// spartan
#include "robot_msgs/CartesianGoalPoint.h"
#include "robot_msgs/CartesianPlanInfo.h"


#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/rigid_body.h>
#include "drake/multibody/rigid_body_frame.h"
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


    cartesian_plan_info_publisher_ = std::make_shared<ros::Publisher>(nh.advertise<robot_msgs::CartesianPlanInfo>("/plan_runner/TaskSpaceStreamingPlan/info", 1));

    world_frame_ = tree_->findFrame("world");
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
    Eigen::Vector3d xyz_ee_goal_; // expressed in world frame
    Eigen::Vector3d xyz_d_ee_goal_; // expressed in world frame
    Eigen::Quaterniond quat_ee_goal_; // expressed in world frame


    // these are the frames we need to keep track of
    std::shared_ptr<RigidBodyFrame<double>> world_frame_;
    std::shared_ptr<RigidBodyFrame<double>> ee_frame_;
    std::shared_ptr<RigidBodyFrame<double>> position_cmd_expressed_in_frame_;
    std::shared_ptr<RigidBodyFrame<double>> linear_velocity_cmd_expressed_in_frame_;
    std::shared_ptr<RigidBodyFrame<double>> angular_velocity_cmd_expressed_in_frame_;

    Eigen::Vector3d linear_velocity_cmd_; // directly read from message
    Eigen::Vector3d angular_velocity_cmd_; // directly read from message

    Eigen::Vector3d setpoint_linear_velocity_cmd_; // directly read from message
    

    bool have_goal_;
    bool have_processed_first_setpoint_message_ = false;
    bool use_ee_velocity_mode_ = false;
    bool setpoint_velocity_mode_ = false;


    std::shared_ptr<ros::Subscriber> setpoint_subscriber_;

    drake::TwistMatrix<double> J_ee_E_; // desired end-effector twist?
    Eigen::Isometry3d H_WE_; // ee to world, current homogeneous transform
    math::RigidTransform<double>
      H_WEr_; // end-effector to world, reference homogeneous transform
  
    Eigen::Vector3d kp_rotation_;
    Eigen::Vector3d kp_translation_;

    
    
    double last_control_update_t_ = 0.;
    robot_msgs::CartesianGoalPoint::ConstPtr cartesian_goal_point_msg_;
    robot_msgs::CartesianPlanInfo cartesian_plan_info_msg_;
    std::shared_ptr<ros::Publisher> cartesian_plan_info_publisher_;

    tf2_ros::TransformBroadcaster br_;
    geometry_msgs::TransformStamped transformStamped_;

};

} // namespace robot_plan_runner
} // namespace drake