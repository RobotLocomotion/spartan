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
                                  const math::RotationMatrixd &R_WE_initial,
                                  const math::RotationMatrixd &R_WE_final,
                                  const Eigen::Vector3d& kp_rotation,
                                  const Eigen::Vector3d& kp_translation,
                                  const std::string &ee_body_name,
                                  double control_period_s = 0.005,
                                  double force_threshold = 20)
      : TrajectoryPlanBase(std::move(tree), xyz_ee_traj),
        cache_(tree_->CreateKinematicsCache()), R_WE_inital_(R_WE_initial),
        R_WE_final_(R_WE_final), kp_rotation_(kp_rotation), kp_translation_(kp_translation),
        ee_body_name_(ee_body_name), control_period_s_(control_period_s),
        force_threshold_(force_threshold), quat_WE_initial_(R_WE_initial.matrix()),
        quat_WE_final_(R_WE_final.matrix()) {
    DRAKE_ASSERT(xyz_ee_traj.rows() == 3);
    idx_ee_ = tree_->FindBodyIndex(ee_body_name_);
    idx_world_ = tree_->FindBodyIndex("world");
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

  /**
   * Computes the orientation trajectory and associated angular velocity vector
  */
  void ComputeOrientationTrajectory(){
    // make sure we go the "short" way around
    if (quat_WE_initial_.dot(quat_WE_final_) < 0){
      quat_WE_final_ = quat_WE_final_.conjugate();
    }

    // compute angular velocity, make sure to only go 0.5 to avoid the corner case
    Eigen::Quaterniond quat_WE_half = quat_WE_initial_.slerp(0.5, quat_WE_final_);

    // figure out angle-axis corresponding to quat_Einit_E
    Eigen::Quaterniond quat_Einit_Ehalf = quat_WE_initial_.inverse() * quat_WE_half;
    Eigen::AngleAxisd angle_axis(quat_Einit_Ehalf);

    // angular velocity of reference trajectory, expressed in Einit frame
    Eigen::Vector3d ang_velocity_WEr_Einit = 2*angle_axis.angle() * angle_axis.axis();

    // angular velocity of reference trajectory, expressed in World frame
    ang_velocity_WEr_W_ = R_WE_inital_ * ang_velocity_WEr_Einit;
  }

private:
  KinematicsCache<double> cache_;
  drake::TwistMatrix<double> J_ee_E_;
  drake::TwistMatrix<double> J_ee_W_;
  Eigen::Isometry3d H_WE_; // ee to world, current homogeneous transform
  math::Transform<double> H_WEr_; // end-effector to world, reference homogeneous transform

  // reference orientation trajectory for EE
  const math::RotationMatrixd R_WE_inital_;
  const math::RotationMatrixd R_WE_final_;
  Eigen::Quaterniond quat_WE_initial_;
  Eigen::Quaterniond quat_WE_final_;

  // angular velocity of reference trajectory, expressed in World frame
  Eigen::Vector3d ang_velocity_WEr_W_;

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