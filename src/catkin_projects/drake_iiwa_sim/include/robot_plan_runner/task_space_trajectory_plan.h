#pragma once
#include <robot_plan_runner/trajectory_plan_base.h>

namespace drake {
namespace robot_plan_runner {

// x_ee_traj \in R^3 is the reference trajectory in world frame which the origin
// of the robot's end effector frame is tracking.
class EndEffectorOriginTrajectoryPlan : public TrajectoryPlanBase {
 public:
  EndEffectorOriginTrajectoryPlan(std::shared_ptr<const RigidBodyTreed> tree,
                                  const PPType &x_ee_traj,
                                  double control_period_s = 0.005)
      : TrajectoryPlanBase(std::move(tree), x_ee_traj),
        cache_(tree_->CreateKinematicsCache()),
        control_period_s_(control_period_s) {
    DRAKE_ASSERT(x_ee_traj.rows() == 3);
    idx_ee_ = tree_->FindBodyIndex("iiwa_link_ee");
    idx_base_ = tree_->FindBodyIndex("base");
    kp.setConstant(50);
  }

  // q, v: current robot configuration/velocity.
  // v_ee_des: desired ee origin velocity (a subset of the ee twist).
  // q_dot_des: deisred joint space velocity.
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
  // x_ee is (a subset of) the end effector pose calculated with forward kinematics.
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded) const override {
    Eigen::VectorXd q = x.head(this->get_num_positions());
    Eigen::VectorXd v = x.tail(this->get_num_velocities());
    cache_.initialize(q, v);
    tree_->doKinematics(cache_);
    J_ee_ = tree_->geometricJacobian(cache_, idx_base_, idx_ee_, idx_base_);
    T_ee_ = tree_->CalcBodyPoseInWorldFrame(cache_, tree_->get_body(idx_ee_));

    Eigen::Vector3d x_ee_ref = traj_.value(t);
    Eigen::Vector3d v_ee_ref = traj_d_.value(t);
    Eigen::Vector3d err_x_ee = x_ee_ref - T_ee_.translation();
    Eigen::Vector3d v_ee_des = kp.array() * err_x_ee.array() + v_ee_ref.array();

    // q_dot_des = J_ee.pseudo_inverse()*v_ee_des
    Eigen::VectorXd q_dot_des =
        J_ee_.bottomRows(3)
            .bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
            .solve(v_ee_des);
    *q_commanded = q + q_dot_des * control_period_s_;
    *v_commanded = q_dot_des; // This is ignored when constructing iiwa_command.
  }

 private:
  mutable KinematicsCache<double> cache_;
  mutable drake::TwistMatrix<double> J_ee_;
  mutable Eigen::Isometry3d T_ee_;
  const double control_period_s_;
  int idx_ee_;
  int idx_base_;
  Eigen::Vector3d kp; // position feedback gain.
};

} // namespace robot_plan_runner
} // namespace drake