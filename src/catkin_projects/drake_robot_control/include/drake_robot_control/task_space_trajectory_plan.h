#pragma once
#include <string>

#include <drake_robot_control/trajectory_plan_base.h>

namespace drake {
namespace robot_plan_runner {

// x_ee_traj \in R^3 is the reference trajectory in world frame which the origin
// of the robot's end effector frame is tracking.
// Orientation of the end effector frame is kept constant when tracking the
// given trajectory.
class EndEffectorOriginTrajectoryPlan : public TrajectoryPlanBase {
public:
  EndEffectorOriginTrajectoryPlan(
      std::shared_ptr<const RigidBodyTreed> tree, const PPType &xyz_ee_traj,
      const Eigen::Ref<const Eigen::Vector3d> &rpy_ref,
      const Eigen::Ref<const Eigen::Vector3d> &force_in_world_frame_ref =
          Eigen::Vector3d::Zero(),
      const std::string &ee_body_name = "iiwa_link_ee",
      double control_period_s = 0.005)
      : TrajectoryPlanBase(std::move(tree), xyz_ee_traj),
        cache_(tree_->CreateKinematicsCache()), rpy_ref_(rpy_ref),
        force_in_world_frame_ref_(force_in_world_frame_ref),
        ee_body_name_(ee_body_name), control_period_s_(control_period_s) {
    DRAKE_ASSERT(xyz_ee_traj.rows() == 3);
    idx_ee_ = tree_->FindBodyIndex(ee_body_name_);
    idx_world_ = tree_->FindBodyIndex("world");
    kp.setConstant(50);
  }

  // q, v: current robot configuration/velocity.
  // v_ee_commanded: commanded ee origin velocity (a subset of the ee twist).
  // q_dot_commanded: commanded joint space velocity.
  // J_ee: geometric Jacobian of the end effector frame expressed in world
  // frame.
  // rows: rows of the J_ee corresponding to the linear velocity of the ee
  // frame.
  // v_ee_des = J_ee[rows] * q_dot_des
  // q_dot_des = J_ee.pseudo_inverse()*v_ee_des
  // q_commanded = q_des = q + q_dot_des * dt, where dt is the control period
  // (iiwa default: 5ms).
  // v_ee_des contains a feedforward term: v_ee_ref,
  // and a feedback term: kp*(x_ee_ref - x_ee).
  // x_ee is (a subset of) the end effector pose calculated with forward
  // kinematics.
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) const override {
    Eigen::VectorXd q = x.head(this->get_num_positions());
    Eigen::VectorXd v = x.tail(this->get_num_velocities());
    cache_.initialize(q, v);
    tree_->doKinematics(cache_);

    rpy_ = tree_->relativeRollPitchYaw(cache_, idx_ee_, idx_world_);
    J_rpy_ =
        tree_->relativeRollPitchYawJacobian(cache_, idx_ee_, idx_world_, true);
    J_ee_ = tree_->geometricJacobian(cache_, idx_world_, idx_ee_, idx_world_);
    T_ee_ = tree_->CalcBodyPoseInWorldFrame(cache_, tree_->get_body(idx_ee_));

    // substitute jacobian w.r.t. roll-pitch-yaw for jacobian w.r.t angular
    // velocity
    J_ee_.topRows(3) = J_rpy_;

    Eigen::Vector3d xyz_ee_ref = traj_.value(t);
    Eigen::Vector3d xyz_d_ee_ref = traj_d_.value(t);
    Eigen::Matrix<double, 6, 1> err, v_ee_ref; // err = [err_rpy_ee; err_xyz_ee]
    err.topRows(3) = rpy_ref_ - rpy_;          // err_rpy_ee
    err.bottomRows(3) = xyz_ee_ref - T_ee_.translation(); // err_xyz_ee
    v_ee_ref.setZero();
    v_ee_ref.bottomRows(3) = xyz_d_ee_ref;

    Eigen::VectorXd v_ee_commanded =
        kp.array() * err.array() + v_ee_ref.array();

    // q_dot_commanded = J_ee.pseudo_inverse()*v_ee_des
    Eigen::VectorXd q_dot_commanded =
        J_ee_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
            .solve(v_ee_commanded);
    *q_commanded = q + q_dot_commanded * control_period_s_;
    *v_commanded =
        q_dot_commanded; // This is ignored when constructing iiwa_command.
    *tau_commanded = J_ee_.bottomRows(3).transpose() * force_in_world_frame_ref_;

    //TODO: set q_commanded constant after t > Plan.duration().
  }

private:
  mutable KinematicsCache<double> cache_;
  mutable drake::TwistMatrix<double> J_ee_;
  mutable Eigen::Isometry3d T_ee_;
  mutable Eigen::Matrix<double, drake::kRpySize, Eigen::Dynamic> J_rpy_;
  mutable Eigen::Vector3d rpy_;
  const Eigen::Vector3d rpy_ref_; // reference roll pitch yaw.
  const Eigen::Vector3d force_in_world_frame_ref_;
  const double control_period_s_;
  const std::string ee_body_name_;
  int idx_ee_;
  int idx_world_;
  Eigen::Matrix<double, 6, 1> kp; // position feedback gain.
};

} // namespace robot_plan_runner
} // namespace drake