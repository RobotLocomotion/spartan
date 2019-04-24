#include <exception>

// ROS
#include <ros/console.h>

#include <Eigen/Dense>
#include <drake_robot_control/task_space_streaming_plan.h>
#include <drake_robot_control/utils.h>

namespace drake {
namespace robot_plan_runner {

typedef drake::TwistVector<double> TwistVectord;

// Current robot state x = [q,v]
// Current time t
void TaskSpaceStreamingPlan::Step(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &tau_external, double t,
    Eigen::VectorXd *const q_commanded, Eigen::VectorXd *const v_commanded,
    Eigen::VectorXd *const tau_commanded) {
  double dt = t - last_control_update_t_;
  last_control_update_t_ = t;
  dt = std::max(std::min(dt, 0.01), 0.0001);

  PlanStatus not_started_status = PlanStatus::NOT_STARTED;
  PlanStatus running_status = PlanStatus::RUNNING;

  plan_status_.compare_exchange_strong(not_started_status, PlanStatus::RUNNING);

  // Lock so we can play with the measured state cache
  std::lock_guard<std::mutex> lock(goal_mutex_);

  // do kinematics so we can check the force guards
  Eigen::VectorXd q = x.head(this->get_num_positions());
  Eigen::VectorXd v = x.tail(this->get_num_positions());
  cache_measured_state_.initialize(q, v);
  tree_->doKinematics(cache_measured_state_);

  // check if the plan has been stopped
  // if so just echo the last command
  if (this->is_stopped() || !this->have_goal_) {
    *q_commanded = q_commanded_prev_;
    *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
    return;
  }

  // Check the external force guards
  if (guard_container_) {
    std::pair<bool, std::pair<double, std::shared_ptr<ForceGuard>>> result =
                                          guard_container_->EvaluateGuards(cache_measured_state_, q,
                                                                           tau_external);

    bool guard_triggered = result.first;

    if (guard_triggered) {
      std::cout << "Force Guard Triggered, commanding in direction of measured position" << std::endl;
      std::cout << "ForceGuardType: " << result.second.second->get_type()
                << std::endl;
      //plan_status_ = PlanStatus::STOPPED_BY_FORCE_GUARD;
      //this->SetPlanFinished();
      *q_commanded = q_commanded_prev_*0.99 + q*0.01;
      q_commanded_prev_ = *q_commanded;
      *tau_commanded = Eigen::VectorXd::Zero(this->get_num_positions());
      return;
    }
  }

  // if we reach here then plan should be in state RUNNING
  // COPIED LARGELY FROM TASK_SPACE_TRAJECTORY_PLAN
  // TODO(gizatt) Refactor to reduce this repeated code.
  // Unfortunately it depends on a large number of internal parameters
  // (lots of frames, gains, and goals), so breaking it out into
  // a function is pretty ugly.
  // Instead of the actual q, use the last commanded q
  Eigen::VectorXd q_measured = x.head(this->get_num_positions());
  q = q_commanded_prev_;

  // std::cout << "Starting control in measured config " << q_measured << " but commanded " << q << std::endl;
  // std::cout << "xyz_ee_goal: " << xyz_ee_goal_ << std::endl;
  // std::cout << "xyz_d_ee_goal: " << xyz_d_ee_goal_ << std::endl;
  // std::cout << "quat_ee_goal_: " << quat_ee_goal_.matrix() << std::endl;

  cache_.initialize(q, v);
  tree_->doKinematics(cache_);

  math::RotationMatrixd R_WEr(quat_ee_goal_);
  math::RotationMatrixd R_ErW = R_WEr.inverse();

  J_ee_E_ = tree_->geometricJacobian(cache_, 0, body_index_ee_frame_, body_index_ee_frame_);
  J_ee_W_ = tree_->geometricJacobian(cache_, 0, body_index_ee_frame_, 0);

  H_WEr_.set_rotation(R_WEr);
  H_WEr_.set_translation(xyz_ee_goal_);
  Eigen::Isometry3d H_WEr = H_WEr_.GetAsIsometry3();

  std::cout << "H_WEr: " << H_WEr.matrix() << std::endl;

  if (body_index_ee_frame_ >= 0){
    H_WE_ = tree_->CalcBodyPoseInWorldFrame(cache_, tree_->get_body(body_index_ee_frame_));
  } else {
    // frames start at -2 and count down.
    H_WE_ = tree_->CalcFramePoseInWorldFrame(cache_, *tree_->get_frames()[-body_index_ee_frame_ - 2]);
  }
  std::cout << "H_WE_: " << H_WE_.matrix() << std::endl;
  Eigen::Isometry3d H_EW = H_WE_.inverse();
  Eigen::Isometry3d H_EEr = H_EW * H_WEr;
  std::cout << "HEEr: " << H_EEr.matrix() << std::endl;

  // Compute the PD part of the control
  // K_{p,w} log_{SO(3)}(R_EEr)
  Eigen::Vector3d log_R_EEr =
      spartan::drake_robot_control::utils::LogSO3(H_EEr.linear());

  TwistVectord twist_pd;
  // we need to do elementwise multiplication, hence the use of array instead
  // of Vector
  twist_pd.head(3) = this->kp_rotation_.array() * log_R_EEr.array();
  twist_pd.tail(3) =
      this->kp_translation_.array() * H_EEr.translation().array();

  // Compute the feed forward part of the control
  // Need twist of reference trajectory with respect to world
  // easiest to compute this as expressed in Er frame,
  // then transform that twist to E frame using adjoint
  TwistVectord T_WEr_Er;

  T_WEr_Er.head(3) = Eigen::Vector3d::Zero(); // hack for now
  T_WEr_Er.tail(3) = R_WEr * xyz_d_ee_goal_;
  
  Eigen::Matrix<double, 6, 6> Ad_H_EEr =
      spartan::drake_robot_control::utils::AdjointSE3(H_EEr.linear(),
                                                      H_EEr.translation());
  TwistVectord T_WEr_E = Ad_H_EEr * T_WEr_Er;

  // Total desired twist
  // Can add these this twists since both expressed in frame E
  TwistVectord T_WE_E_cmd = twist_pd + T_WEr_E;

  // q_dot_cmd = J_ee.pseudo_inverse()*T_WE_E_cmd
  auto svd = J_ee_E_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  // When computing the pseudoinverse, this ignores all singular
  // values smaller than this threshold. This is raised
  // from the Eigen default to create less jerky movements near
  // singularities.
  svd.setThreshold(0.01);
  Eigen::VectorXd q_dot_cmd = svd.solve(T_WE_E_cmd);
  std::cout << "Final q dot cmd: " << q_dot_cmd << std::endl;
  *q_commanded = q + q_dot_cmd * dt;
  *v_commanded = q_dot_cmd; // This is ignored when constructing iiwa_command.

  bool unsafe_command = Eigen::isnan(q_commanded->array()).any();
  if (unsafe_command) {
    std::cout << "\n\nunsafe command caught inside Step()" << std::endl;
    std::cout << "q_commanded:\n" << *q_commanded << std::endl;
    std::cout << "q_dot_cmd:\n" << q_dot_cmd << std::endl;
    std::cout << "q:\n" << q << std::endl;

    std::cout << "\nT_WE_E_cmd:\n" << T_WE_E_cmd << std::endl;
  }

  bool debug = false;
  if (debug) {
    std::cout << "\n-------\n";
    std::cout << "\nT_WE_E_cmd:\n" << T_WE_E_cmd << std::endl;

    Eigen::Isometry3d H_ErE = H_EEr.inverse();
    std::cout << "H_ErE.translation() " << H_ErE.translation() << std::endl;
  }
  // debugging prints for when things are nan . . .
}

void TaskSpaceStreamingPlan::HandleSetpoint(
    const robot_msgs::CartesianGoalPoint::ConstPtr& msg) {
  if (this->is_stopped()){
    std::cout << "In callback, but plan is stopped... forcefully unregistering." << std::endl;
    this->setpoint_subscriber_->shutdown();
  }
  std::lock_guard<std::mutex> lock(goal_mutex_);
  //std::cout << "Starting to handle setpoint... " << std::endl;
  // Extract the body index in the RBT that this msg
  // is referring to.
  // These will throw if the frame isn't unique or doesn't exist.
  body_index_ee_goal_ = tree_->findFrame(
    msg->xyz_point.header.frame_id)->get_frame_index();
  body_index_ee_frame_ = tree_->findFrame(msg->ee_frame_id)->get_frame_index();

  tree_->doKinematics(cache_measured_state_);

  xyz_ee_goal_ = Eigen::Vector3d(msg->xyz_point.point.x,
                                 msg->xyz_point.point.y,
                                 msg->xyz_point.point.z);
  // Transform to world frame at last observed robot posture
  auto R = tree_->relativeTransform(
    cache_measured_state_, 0, body_index_ee_goal_)
    .matrix().block<3, 3>(0, 0);
  xyz_ee_goal_ = tree_->transformPoints(
    cache_measured_state_, xyz_ee_goal_, body_index_ee_goal_, 0);
  xyz_d_ee_goal_ = Eigen::Vector3d(msg->xyz_d_point.x,
                                   msg->xyz_d_point.y,
                                   msg->xyz_d_point.z);
  xyz_d_ee_goal_ = R*xyz_d_ee_goal_;
  quat_ee_goal_ =  Eigen::Quaterniond(msg->quaternion.w,
                                     msg->quaternion.x,
                                     msg->quaternion.y,
                                     msg->quaternion.z);
  quat_ee_goal_ = 
    (drake::math::RotationMatrixd(R)*
     drake::math::RotationMatrixd(quat_ee_goal_)).ToQuaternion();

  kp_rotation_ = Eigen::Vector3d(msg->gain.rotation.x,
                                 msg->gain.rotation.y,
                                 msg->gain.rotation.z);
  kp_translation_ = Eigen::Vector3d(msg->gain.translation.x,
                                    msg->gain.translation.y,
                                    msg->gain.translation.z);
  have_goal_ = true;
  //std::cout << "Finished handling setpoint from config " << q_commanded_prev_ << std::endl;
}

} // namespace robot_plan_runner
} // namespace drake