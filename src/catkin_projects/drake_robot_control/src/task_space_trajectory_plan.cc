#include <drake_robot_control/task_space_trajectory_plan.h>
#include <drake_robot_control/utils.h>
#include <Eigen/Dense>

//ROS
#include <ros/console.h>


namespace drake {
namespace robot_plan_runner {

/// Notation
/// H_JI: is the homogeneous transform from I to J
/// T_JI_K: twist of I with respect to J expressed in frame K

/// This plan implements a task space controller
/// The treatment here follows the notation of [1] and [2]
/// [1] Design of a Momentum based Control Framework and its application
/// to the Humanoid Robot Atlas
/// [2] Proportional Derivative (PD) Control on the Euclidean Group
/// E_r is the reference trajectory for the body frame
/// E is the current frame of the body frame
/// Since the kuka is velocity controlled this is a first order system
/// The error term is given be e = (H_WEr)^{-1} H_WE
/// H_WE represents the homogenous transform from E to W
/// The dynamics of the error term are
/// V^e = V - Ad_{H_WE_inv}(V_d) where V_d is the velocity of the reference
/// trajectory. A stabilizing controller for V^e just drives it to the origin/identity
/// using a PD controller
/// (rotation)  K_{p,w} log_{SO(3)}(R_EEr)
/// (translation) K_{p,v} p_{E_r}^E
/// and also feed forward
/// (feed-forward) T_EW_Er
/// T_EW_E = PD + feed-forward


typedef drake::TwistVector<double> TwistVectord;


  void EndEffectorOriginTrajectoryPlan::Step(const Eigen::Ref<const Eigen::VectorXd> &x,
            const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
            Eigen::VectorXd *const q_commanded,
            Eigen::VectorXd *const v_commanded,
            Eigen::VectorXd *const tau_commanded) {

    DRAKE_ASSERT(t >= 0);

    PlanStatus not_started_status = PlanStatus::NOT_STARTED;
    PlanStatus running_status = PlanStatus::RUNNING;

    if (plan_status_.compare_exchange_strong(not_started_status, PlanStatus::RUNNING) ){
     std::cout << "plan status was NOT_STARTED, setting it to RUNNING" << std::endl;
    }

    Eigen::VectorXd q = x.head(this->get_num_positions());
    Eigen::VectorXd v = x.tail(this->get_num_velocities());
    Eigen::Vector3d xyz_ee_ref = traj_.value(t);
    Eigen::Vector3d xyz_d_ee_ref = traj_d_.value(t);

    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());

    cache_.initialize(q, v);
    tree_->doKinematics(cache_);
    J_ee_E_ = tree_->geometricJacobian(cache_, idx_world_, idx_ee_, idx_ee_);
    J_ee_W_ = tree_->geometricJacobian(cache_, idx_world_, idx_ee_, idx_world_);



    if (this->get_plan_status() == PlanStatus::RUNNING) {

      // check if force threshold was tripped
      if (t < this->duration()) {
        TwistVectord ee_force_W_;
        ee_force_W_.head(3).setZero(); // no external moment
        ee_force_W_.tail(3) =
            xyz_d_ee_ref / xyz_d_ee_ref.norm() * force_threshold_;
        Eigen::VectorXd joint_torque = J_ee_W_.transpose() * ee_force_W_;

        if (tau_external.norm() > joint_torque.norm()) {
          if (plan_status_.compare_exchange_strong(running_status, PlanStatus::STOPPPED_BY_FORCE_THRESHOLD)) {
            this->SetPlanFinished();
            q_command_final_ = q_commanded_prev_;
            std::cout << "Norm of joint torque exceeding threshold "
                      << joint_torque.norm() << std::endl;
          }
        }
      }
      else {
        // if we arrive here, it means that plan status was
        // RUNNING and t > this->duration()
        if (plan_status_.compare_exchange_strong(running_status, PlanStatus::FINISHED_NORMALLY)){
          ROS_INFO("plan finished normally");
          // notify the condition variable
          q_command_final_ = q_commanded_prev_;
          this->SetPlanFinished();
        }
      }
    }



    if (this->get_plan_status() == PlanStatus::RUNNING){

      H_WEr_.set_rotation(R_WEr_);
      H_WEr_.set_translation(xyz_ee_ref);
      Eigen::Isometry3d H_WEr = H_WEr_.GetAsIsometry3();

      H_WE_ = tree_->CalcBodyPoseInWorldFrame(cache_, tree_->get_body(idx_ee_));

      Eigen::Isometry3d H_EW = H_WE_.inverse();
      Eigen::Isometry3d H_EEr = H_EW * H_WEr;

      // Compute the PD part of the control
      // K_{p,w} log_{SO(3)}(R_EEr)
      Eigen::Vector3d log_R_EEr = spartan::drake_robot_control::utils::LogSO3(H_EEr.linear());

      TwistVectord twist_pd;
      // we need to do elementwise multiplication, hence the use of array instead
      // of Vector
      twist_pd.head(3) = this->kp_rotation_.array() * log_R_EEr.array();
      twist_pd.tail(3) = this->kp_translation_.array() * H_EEr.translation().array();


      // Compute the feed forward part of the control
      // For now the feed-forward rotation part is set to zero
      TwistVectord T_WEr_W;
      T_WEr_W.head(3).setZero();
      T_WEr_W.tail(3) = xyz_d_ee_ref; // if angular velocity was non-zero this would have to change

      // Convert this twist to be expressed in body frame.
      // To convert a twist use the adjoint map
      Eigen::Matrix<double, 6, 6> Ad_H_EW =
          spartan::drake_robot_control::utils::AdjointSE3(H_EW.linear(), H_EW.translation());
      TwistVectord T_WEr_E = Ad_H_EW * T_WEr_W;


      // Total desired twist
      // Can add these this twists since both expressed in frame E
      TwistVectord T_WE_E_cmd = twist_pd + T_WEr_E;

      // q_dot_cmd = J_ee.pseudo_inverse()*T_WE_E_cmd
      Eigen::VectorXd q_dot_cmd =
          J_ee_E_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
              .solve(T_WE_E_cmd);
      *q_commanded = q + q_dot_cmd * control_period_s_;
      *v_commanded = q_dot_cmd; // This is ignored when constructing iiwa_command.


      bool unsafe_command = Eigen::isnan(q_commanded->array()).any();
      if (unsafe_command){
        std::cout << "\n\nunsafe command caught inside Step()" << std::endl;
        std::cout << "q_commanded:\n" << *q_commanded << std::endl;
        std::cout << "q_dot_cmd:\n" << q_dot_cmd << std::endl;
        std::cout << "q:\n" << q << std::endl;

        std::cout << "\nT_WE_E_cmd:\n" << T_WE_E_cmd << std::endl;
      }

    } else {
      // this means plan_status_ != RUNNING
      *q_commanded = q_commanded_prev_; // just echo the current commanded position and torque
      *v_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
      *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
    }


    // debugging prints for when things are nan . . .

  }
} // namespace robot_plan_runner
} // namespace drake