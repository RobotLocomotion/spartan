#pragma once
#include <string>

#include <drake_robot_control/trajectory_plan_base.h>

namespace drake {
namespace robot_plan_runner {

/*
 *  TODO: the implementation in this Plan has a number of drawbacks.
 *  1. roll pitch yaw is not an ideal representation of rotation. The same
 *  EE orientation could have very different rpy angles. Therefore doing PD
 * control directly on rpy is very bad.
 *
 *  2. The robot looks precarious when commanded to move near sigularities.
 *
 *  3. When multiple dq's exist, the robot should choose one that is close to a
 * nominal pose. For example, we prefer the robot to move its "elbow" away,
 * not towards the table when it moves its ee in Cartesian space.
 *
 * A reliable Cartesion space controller therefore could be solving QP's instead of
 * lease squares.
 */

// xyz_ee_traj \in R^3 is the reference trajectory in world frame which the
// origin of the robot's end effector frame is tracking.
// Roll pitch yaw of the ee frame is measured and stored in rpy_ref_ when the
// plan is constructed and kept constant during the plan's execution.

class EndEffectorOriginTrajectoryPlan : public TrajectoryPlanBase {
public:
  EndEffectorOriginTrajectoryPlan(
      std::shared_ptr<const RigidBodyTreed> tree, const PPType &xyz_ee_traj,
      const Eigen::Ref<const Eigen::Vector3d> &rpy_ref,
      const std::string &ee_body_name, double control_period_s = 0.005)
      : TrajectoryPlanBase(std::move(tree), xyz_ee_traj),
        cache_(tree_->CreateKinematicsCache()), rpy_ref_(rpy_ref),
        ee_body_name_(ee_body_name), control_period_s_(control_period_s) {
    DRAKE_ASSERT(xyz_ee_traj.rows() == 3);
    DRAKE_ASSERT(rpy_ref.rows() == 3);
    idx_ee_ = tree_->FindBodyIndex(ee_body_name_);
    idx_world_ = tree_->FindBodyIndex("world");
    kp_.setConstant(50);
    is_finished_ = false;
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
  // and a feedback term: kp_*(x_ee_ref - x_ee).
  // x_ee is (a subset of) the end effector pose calculated with forward
  // kinematics.
  void Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) override {
    Eigen::VectorXd q = x.head(this->get_num_positions());
    Eigen::VectorXd v = x.tail(this->get_num_velocities());

    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());

    // set q_commanded constant after t > Plan.duration().
    if (!is_finished_ && t > this->duration()) {
      is_finished_ = true;
      q_final_ = q;
      std::cout << "Plan finished, holding position:\n"
                << q_final_ << std::endl;
    }

    if (is_finished_) {
      *q_commanded = q_final_;
      *v_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
    } else {
      cache_.initialize(q, v);
      tree_->doKinematics(cache_);

      rpy_ = tree_->relativeRollPitchYaw(cache_, idx_ee_, idx_world_);
      J_rpy_ = tree_->relativeRollPitchYawJacobian(cache_, idx_ee_, idx_world_,
                                                   true);
      J_ee_ = tree_->geometricJacobian(cache_, idx_world_, idx_ee_, idx_world_);
      T_ee_ = tree_->CalcBodyPoseInWorldFrame(cache_, tree_->get_body(idx_ee_));

      // substitute jacobian w.r.t. roll-pitch-yaw for jacobian w.r.t angular
      // velocity
      J_ee_.topRows(3) = J_rpy_;

      Eigen::Vector3d xyz_ee_ref = traj_.value(t);
      Eigen::Vector3d xyz_d_ee_ref = traj_d_.value(t);
      Eigen::Matrix<double, 6, 1> err,
          v_ee_ref;                     // err = [err_rpy_ee; err_xyz_ee]
      err.topRows(3) = rpy_ref_ - rpy_; // err_rpy_ee
      err.bottomRows(3) = xyz_ee_ref - T_ee_.translation(); // err_xyz_ee
      v_ee_ref.setZero();
      v_ee_ref.bottomRows(3) = xyz_d_ee_ref;

      Eigen::VectorXd v_ee_commanded =
          kp_.array() * err.array() + v_ee_ref.array();

      // q_dot_commanded = J_ee.pseudo_inverse()*v_ee_des
      Eigen::VectorXd q_dot_commanded =
          J_ee_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
              .solve(v_ee_commanded);
      *q_commanded = q + q_dot_commanded * control_period_s_;
      *v_commanded =
          q_dot_commanded; // This is ignored when constructing iiwa_command.
    }
  }

private:
  KinematicsCache<double> cache_;
  drake::TwistMatrix<double> J_ee_;
  Eigen::Isometry3d T_ee_;
  Eigen::Matrix<double, drake::kRpySize, Eigen::Dynamic> J_rpy_;
  Eigen::Vector3d rpy_;
  bool is_finished_;
  const Eigen::Vector3d rpy_ref_; // reference roll pitch yaw.
  const double control_period_s_;
  const std::string ee_body_name_;
  int idx_ee_;
  int idx_world_;
  Eigen::Matrix<double, 6, 1> kp_; // position feedback gain.
  // final robot configuration after the plan is completed.
  Eigen::VectorXd q_final_;
};

} // namespace robot_plan_runner
} // namespace drake