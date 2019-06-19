#include <exception>

// ROS
#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>

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

  // do kinematics for previously commanded control
  cache_.initialize(q_commanded_prev_, v);
  tree_->doKinematics(cache_);

  

  J_ee_E_ = tree_->geometricJacobian(cache_, 0, body_index_ee_frame_, body_index_ee_frame_);
  J_ee_W_ = tree_->geometricJacobian(cache_, 0, body_index_ee_frame_, 0);
  H_WE_ = tree_->CalcFramePoseInWorldFrame(cache_, *this->ee_frame_);

  
  // We want to compute Twist command for end-effector. Frames are as follows
  // from: end-effector (E),
  // to: world (W)
  // expressed in: end-effector (E)
  // start with it set to zero, will fill it in later depending on what mode we are in
  // Note EE frame is computed using forward kinematics from q_commanded_prev_ not q_measured_
  
  // drake::TwistVectord
  TwistVectord T_WE_E_cmd;

  if (this->use_ee_velocity_mode_){ // velocity goal mode
    // here we just need to apply the correct rotation matrices.
    // Note that we are using velocities not twists

    auto R_angular = tree_->relativeTransform(
    cache_, this->ee_frame_->get_frame_index(), this->angular_velocity_cmd_expressed_in_frame_->get_frame_index()).linear();

    auto R_linear = tree_->relativeTransform(
    cache_, this->ee_frame_->get_frame_index(), this->linear_velocity_cmd_expressed_in_frame_->get_frame_index()).linear();

    T_WE_E_cmd.head(3) = R_angular * this->angular_velocity_cmd_;
    T_WE_E_cmd.tail(3) = R_linear * this->linear_velocity_cmd_;

  } else{ // position goal mode    

    math::RotationMatrixd R_WEr(quat_ee_goal_);
    math::RotationMatrixd R_ErW = R_WEr.inverse();

    H_WEr_.set_rotation(R_WEr);
    H_WEr_.set_translation(xyz_ee_goal_);
    Eigen::Isometry3d H_WEr = H_WEr_.GetAsIsometry3();

    Eigen::Isometry3d H_EW = H_WE_.inverse();
    Eigen::Isometry3d H_EEr = H_EW * H_WEr;

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
    T_WE_E_cmd = twist_pd + T_WEr_E;
  }


  // alternatively do direct velocity command

  // q_dot_cmd = J_ee.pseudo_inverse()*T_WE_E_cmd
  auto svd = J_ee_E_.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  // When computing the pseudoinverse, this ignores all singular
  // values smaller than this threshold. This is raised
  // from the Eigen default to create less jerky movements near
  // singularities.
  svd.setThreshold(0.01);
  Eigen::VectorXd q_dot_cmd = svd.solve(T_WE_E_cmd);
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

  // construct the ROS message for use in behavior cloning
  auto & msg  = cartesian_plan_info_msg_;
  msg.header.stamp = ros::Time::now();
  msg.cartesian_goal_point_cmd = *this->cartesian_goal_point_msg_;

  msg.q_measured.resize(this->get_num_positions());
  msg.q_cmd_prev.resize(this->get_num_positions());
  
  msg.q_cmd.resize(this->get_num_positions());

  for(int i = 0; i < this->get_num_positions(); i++){
    msg.q_measured[i] = q_measured(i);
    msg.q_cmd_prev[i] = q_commanded_prev_(i);
    msg.q_cmd[i] = (*q_commanded)(i);
  }

  msg.v_cmd.resize(this->get_num_velocities());
  msg.v_measured.resize(this->get_num_velocities());
  for(int i = 0; i < this->get_num_velocities(); i++){
    msg.v_cmd[i] = (*v_commanded)(i);
    msg.v_measured[i] = v(i);
  }

  msg.ee_twist_cmd_expressed_in_body.angular.x = T_WE_E_cmd(0);
  msg.ee_twist_cmd_expressed_in_body.angular.y = T_WE_E_cmd(1);
  msg.ee_twist_cmd_expressed_in_body.angular.z = T_WE_E_cmd(2);

  msg.ee_twist_cmd_expressed_in_body.linear.x = T_WE_E_cmd(3);
  msg.ee_twist_cmd_expressed_in_body.linear.y = T_WE_E_cmd(4);
  msg.ee_twist_cmd_expressed_in_body.linear.z = T_WE_E_cmd(5);

  auto & angular_world_frame = H_WE_.linear() * T_WE_E_cmd.head(3);
  auto & linear_world_frame = H_WE_.linear() * T_WE_E_cmd.tail(3);

  msg.linear_velocity_cmd_expressed_in_world.x = linear_world_frame(0);
  msg.linear_velocity_cmd_expressed_in_world.y = linear_world_frame(1);
  msg.linear_velocity_cmd_expressed_in_world.z = linear_world_frame(2);

  msg.angular_velocity_cmd_expressed_in_world.x = angular_world_frame(0);
  msg.angular_velocity_cmd_expressed_in_world.y = angular_world_frame(1);
  msg.angular_velocity_cmd_expressed_in_world.z = angular_world_frame(2);


  // H_W_E, this uses q_cmd
  auto & tf_msg_q_cmd = msg.end_effector_to_world_w_q_cmd;
  tf_msg_q_cmd.header.frame_id = "base";
  tf_msg_q_cmd.header.stamp = ros::Time::now();
  tf_msg_q_cmd.child_frame_id = msg.cartesian_goal_point_cmd.ee_frame_id;


  tf_msg_q_cmd.transform.translation.x = H_WE_.translation()(0);
  tf_msg_q_cmd.transform.translation.y = H_WE_.translation()(1);
  tf_msg_q_cmd.transform.translation.z = H_WE_.translation()(2);

  Eigen::Quaterniond H_WE_quat = Eigen::Quaterniond(H_WE_.linear());
  tf_msg_q_cmd.transform.rotation.w = H_WE_quat.w();
  tf_msg_q_cmd.transform.rotation.x = H_WE_quat.x();
  tf_msg_q_cmd.transform.rotation.y = H_WE_quat.y();
  tf_msg_q_cmd.transform.rotation.z = H_WE_quat.z();

  // H_W_E with q_meas
  H_WE_ = tree_->CalcFramePoseInWorldFrame(cache_measured_state_, *this->ee_frame_);

  auto & tf_msg_q_meas = msg.end_effector_to_world;
  tf_msg_q_meas.header.frame_id = "base";
  tf_msg_q_meas.header.stamp = ros::Time::now();
  tf_msg_q_meas.child_frame_id = msg.cartesian_goal_point_cmd.ee_frame_id;


  tf_msg_q_meas.transform.translation.x = H_WE_.translation()(0);
  tf_msg_q_meas.transform.translation.y = H_WE_.translation()(1);
  tf_msg_q_meas.transform.translation.z = H_WE_.translation()(2);

  H_WE_quat = Eigen::Quaterniond(H_WE_.linear());
  tf_msg_q_meas.transform.rotation.w = H_WE_quat.w();
  tf_msg_q_meas.transform.rotation.x = H_WE_quat.x();
  tf_msg_q_meas.transform.rotation.y = H_WE_quat.y();
  tf_msg_q_meas.transform.rotation.z = H_WE_quat.z();


  this->cartesian_plan_info_publisher_->publish(msg);
}

void TaskSpaceStreamingPlan::HandleSetpoint(
    const robot_msgs::CartesianGoalPoint::ConstPtr& msg) {
  if (this->is_stopped()){
    std::cout << "In callback, but plan is stopped... forcefully unregistering." << std::endl;
    this->setpoint_subscriber_->shutdown();
  }

  
  goal_mutex_.lock();
  this->cartesian_goal_point_msg_ = msg; // store message for later use
  
  //std::cout << "Starting to handle setpoint... " << std::endl;
  // Extract the body index in the RBT that this msg
  // is referring to.
  // These will throw if the frame isn't unique or doesn't exist.

  ee_frame_ = tree_->findFrame(msg->ee_frame_id);
  body_index_ee_frame_ = ee_frame_->get_frame_index();
  tree_->doKinematics(cache_measured_state_);


  // parse the velocity goals
  this->use_ee_velocity_mode_ = msg->use_end_effector_velocity_mode;
  if(this->use_ee_velocity_mode_){
    
    linear_velocity_cmd_expressed_in_frame_ = tree_->findFrame(msg->linear_velocity.header.frame_id);
    angular_velocity_cmd_expressed_in_frame_ = tree_->findFrame(msg->angular_velocity.header.frame_id);

    this->linear_velocity_cmd_ = Eigen::Vector3d(msg->linear_velocity.vector.x,
                                 msg->linear_velocity.vector.y,
                                 msg->linear_velocity.vector.z);

    this->angular_velocity_cmd_ = Eigen::Vector3d(msg->angular_velocity.vector.x,
                                 msg->angular_velocity.vector.y,
                                 msg->angular_velocity.vector.z);
  } else{

  ee_goal_expressed_in_frame_ = tree_->findFrame(
          msg->xyz_point.header.frame_id);

  body_index_ee_goal_ = ee_goal_expressed_in_frame_->get_frame_index();
  


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

  }


  have_goal_ = true;
  


  // if (!this->use_ee_velocity_mode_){
  //   // publish out the goal frame via tf


  //   this->transformStamped_.header.stamp = ros::Time::now();
  //   this->transformStamped_.header.frame_id = "base";
  //   this->transformStamped_.child_frame_id = "teleop_frame";
  //   this->transformStamped_.transform.translation.x = xyz_ee_goal_(0);
  //   this->transformStamped_.transform.translation.y = xyz_ee_goal_(1);
  //   this->transformStamped_.transform.translation.z = xyz_ee_goal_(2);
  //   this->transformStamped_.transform.rotation.x = quat_ee_goal_.x();
  //   this->transformStamped_.transform.rotation.y = quat_ee_goal_.y();
  //   this->transformStamped_.transform.rotation.z = quat_ee_goal_.z();
  //   this->transformStamped_.transform.rotation.w = quat_ee_goal_.w();

  //   this->br_.sendTransform(this->transformStamped_);
  // } 

  goal_mutex_.unlock();
}

} // namespace robot_plan_runner
} // namespace drake