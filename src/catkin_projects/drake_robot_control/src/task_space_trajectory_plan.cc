#include <drake_robot_control/task_space_trajectory_plan.h>


namespace drake {
namespace robot_plan_runner {

  void EndEffectorOriginTrajectoryPlan::Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) {
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
} // namespace robot_plan_runner
} // namespace drake