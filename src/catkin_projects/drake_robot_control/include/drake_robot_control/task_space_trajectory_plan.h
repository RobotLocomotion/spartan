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
 *  2. The robot looks precarious when commanded to move near sigularities.
 *
 *  3. When multiple dq's exist, the robot should choose one that is close to a
 * nominal pose. For example, we prefer the robot to move its "elbow" away,
 * not towards the table when it moves its ee in Cartesian space.
 *
 * A reliable Cartesion space controller therefore could be solving QP's instead
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
    kp_rotation_.setConstant(25);
    kp_translation_.setConstant(50);
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
            Eigen::VectorXd *const tau_commanded) override {
    Eigen::VectorXd q = x.head(this->get_num_positions());
    Eigen::VectorXd v = x.tail(this->get_num_velocities());
    Eigen::Vector3d xyz_ee_ref = traj_.value(t);
    Eigen::Vector3d xyz_d_ee_ref = traj_d_.value(t);

    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());

    cache_.initialize(q, v);
    tree_->doKinematics(cache_);
    J_ee_E_ = tree_->geometricJacobian(cache_, idx_world_, idx_ee_, idx_ee_);
    J_ee_W_ = tree_->geometricJacobian(cache_, idx_world_, idx_ee_, idx_world_);

    if (!is_finished_) {
      if (t < this->duration()) {
        TwistVector<double> ee_force_W_;
        ee_force_W_.head(3).setZero(); // no external moment
        ee_force_W_.tail(3) =
            xyz_d_ee_ref / xyz_d_ee_ref.norm() * force_threshold_;
        Eigen::VectorXd joint_torque = J_ee_W_.transpose() * ee_force_W_;
        if (tau_external.norm() > joint_torque.norm()) {
          is_finished_ = true;
          q_final_ = q;
          std::cout << "Norm of joint torque exceeding threshold "
                    << joint_torque.norm() << ", holding current position:\n"
                    << q_final_ << std::endl;
        }
      } else {
        // t >= duration, plan finished normally.
        is_finished_ = true;
        q_final_ = q;
        std::cout << "Plan finished, holding position:\n"
                  << q_final_ << std::endl;
      }
    }


    if (is_finished_) {
      *q_commanded = q_final_;
      *v_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
    } else {
      H_WEr_.set_rotation(R_WEr_);
      H_WEr_.set_translation(xyz_ee_ref);
      Eigen::Isometry3d H_WEr = H_WEr_.GetAsIsometry3();

      H_WE_ = tree_->CalcBodyPoseInWorldFrame(cache_, tree_->get_body(idx_ee_));

      Eigen::Isometry3d H_EW = H_WE_.inverse();
      Eigen::Matrix<double, 6, 6> Adjoint_EW;
      Eigen::Vector3d p = H_EW.translation();
      Eigen::Matrix3d p_head;
      p_head << 0, -p(2), p(1), //
          p(2), 0, -p(0),       //
          -p(1), p(0), 0;
      Adjoint_EW.topLeftCorner(3, 3) = H_EW.linear();
      Adjoint_EW.topRightCorner(3, 3).setZero();
      Adjoint_EW.bottomLeftCorner(3, 3) = p_head * H_EW.linear();
      Adjoint_EW.bottomRightCorner(3, 3) = H_EW.linear();

      Eigen::Isometry3d H_EEr = H_EW * H_WEr;

      TwistVector<double> T_WEr_W;
      T_WEr_W.head(3).setZero();
      T_WEr_W.tail(3) = xyz_d_ee_ref;
      TwistVector<double> T_WEr_E = Adjoint_EW * T_WEr_W;

      const Eigen::Matrix3d R_EEr = H_EEr.linear();
      const double phi = std::acos(0.5 * (R_EEr.trace() - 1));
      double phi_divided_by_sin_phi;
      if (std::abs(phi) < 1e-4) {
        phi_divided_by_sin_phi = 1;
      } else {
        phi_divided_by_sin_phi = phi / std::sin(phi);
      }

      const Eigen::Matrix3d log_R_EEr =
          0.5 * phi_divided_by_sin_phi * (R_EEr - R_EEr.transpose());
      const Eigen::Vector3d v_log_R_EEr(log_R_EEr(2, 1), log_R_EEr(0, 2),
                                        log_R_EEr(1, 0));

      TwistVector<double> T_WE_E_cmd;
      T_WE_E_cmd.head(3) = kp_rotation_.array() * v_log_R_EEr.array();
      T_WE_E_cmd.tail(3) =
          kp_translation_.array() * H_EEr.translation().array();

      T_WE_E_cmd += T_WEr_E;

      // q_dot_cmd = J_ee.pseudo_inverse()*T_WE_E_cmd
      Eigen::VectorXd q_dot_cmd =
          J_ee_E_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
              .solve(T_WE_E_cmd);
      *q_commanded = q + q_dot_cmd * control_period_s_;
      *v_commanded =
          q_dot_cmd; // This is ignored when constructing iiwa_command.
    }
  }

private:
  KinematicsCache<double> cache_;
  drake::TwistMatrix<double> J_ee_E_;
  drake::TwistMatrix<double> J_ee_W_;
  Eigen::Isometry3d H_WE_;
  math::Transform<double> H_WEr_;
  bool is_finished_;
  const math::RotationMatrixd R_WEr_;
  const double control_period_s_;
  const std::string ee_body_name_;
  int idx_ee_;
  int idx_world_;
  Eigen::Vector3d kp_rotation_;
  Eigen::Vector3d kp_translation_;
  // final robot configuration after the plan is completed.
  Eigen::VectorXd q_final_;
  const double force_threshold_;
};

} // namespace robot_plan_runner
} // namespace drake