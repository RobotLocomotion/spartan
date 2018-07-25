#pragma once
#include <string>

#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/transform.h>
#include <drake_robot_control/trajectory_plan_base.h>

namespace drake {
namespace robot_plan_runner {

/*
 *  TODO: the implementation in this Plan has a number of drawbacks.
 *  1. For smoother rotation commands, take in a quaternion slerp,
 *    instead of a fixed reference rotation matrix.
 *
 *  2. The robot looks precarious when commanded to move near singularities.
 *
 *  3. When multiple dq's exist, the robot should choose one that is close to a
 * nominal pose. For example, we prefer the robot to move its "elbow" away,
 * not towards the table when it moves its ee in Cartesian space.
 *
 * A reliable Cartesian space controller therefore could be solving QP's instead
 * of least squares.
 */

// xyz_ee_traj \in R^3 is the reference trajectory in world frame which the
// origin of the robot's end effector frame is tracking.
// Notation for reference frames used in this Plan:
// W: world frame
// E: end effector frame
// Er: end effector reference frame (the one E is tracking).
// Examples of vectors and transforms:
// [v]_Er: a vector v expressed in frame Er
// R_WEr is the transformation from frame Er to frame W: [v]_W = R_WEr*[v]_Er
// T_WEr_W: twist of frame Er w.r.t frame W, expressed in frame W.

// Roll pitch yaw of the ee frame is measured and stored in rpy_ref_ when the
// plan is constructed and kept constant during the plan's execution.

// This Plan uses double-geodesic PD control law (whatever that means...)
// to track rotation. For more information, please refer page 7 of Twan's paper:
// Design of a momentum-based control framework and application to the humandoid
// robot atlas.

// Force threshold is in Newtons.

class EndEffectorOriginTrajectoryPlan : public TrajectoryPlanBase {
public:
  EndEffectorOriginTrajectoryPlan(std::shared_ptr<const RigidBodyTreed> tree,
                                  const PPType &xyz_ee_traj,
                                  const math::RotationMatrixd &R_WEr,
                                  const std::string &ee_body_name,
                                  double control_period_s = 0.005,
                                  double force_threshold = 20)
      : TrajectoryPlanBase(std::move(tree), xyz_ee_traj),
        cache_(tree_->CreateKinematicsCache()), R_WEr_(R_WEr),
        ee_body_name_(ee_body_name), control_period_s_(control_period_s),
        force_threshold_(force_threshold) {
    DRAKE_ASSERT(xyz_ee_traj.rows() == 3);
    idx_ee_ = tree_->FindBodyIndex(ee_body_name_);
    idx_world_ = tree_->FindBodyIndex("world");
    kp_rotation_.setConstant(5);
    kp_translation_.setConstant(10);
    is_finished_ = false;
  }

  // q, v: current robot configuration/velocity.
  // v_ee_commanded: commanded ee origin velocity (a subset of the ee twist).
  // J_ee_E: geometric Jacobian of the end effector frame expressed in EE frame.
  // frame.
  // T_WE_E_cmd = J_ee_E * q_dot_cmd
  // q_dot_cmd = J_ee.pseudo_inverse()*v_ee_des
  // q_commanded = q_des = q + q_dot_des * dt, where dt is the control period
  // (iiwa default: 5ms).
  // T_WE_E_cmd contains a feedforward term: T_WEr_E,
  // and a feedback term: (see Twan's paper).
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) override;

private:
  KinematicsCache<double> cache_;
  drake::TwistMatrix<double> J_ee_E_;
  drake::TwistMatrix<double> J_ee_W_;
  Eigen::Isometry3d H_WE_; // ee to world, current homogeneous transform
  math::Transform<double> H_WEr_; // end-effector to world, reference homogeneous transform
  const math::RotationMatrixd R_WEr_; // end-effector to world, reference rotation
  const double control_period_s_;
  const std::string ee_body_name_;
  int idx_ee_;
  int idx_world_;
  Eigen::Vector3d kp_rotation_;
  Eigen::Vector3d kp_translation_;
  // final robot configuration after the plan is completed.
  Eigen::VectorXd q_command_final_;
  const double force_threshold_;
};

} // namespace robot_plan_runner
} // namespace drake