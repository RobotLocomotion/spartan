#include <boost/format.hpp>
#include <math.h>

#include <drake/math/roll_pitch_yaw.h>
#include <drake_robot_control/plan_runner.h>
#include <yaml-cpp/yaml.h>

#include "common_utils/system_utils.h"
#include "drake_robot_control/force_guard.h"
#include "drake_robot_control/utils.h"

// ROS
// #include <tf_conversions/tf2_eigen.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace drake {
namespace robot_plan_runner {

double ToRadians(double degrees) {
  return degrees * M_PI / 180.;
}

typedef spartan::drake_robot_control::ForceGuard ForceGuard;
typedef spartan::drake_robot_control::TotalExternalTorqueGuard
    TotalExternalTorqueGuard;
typedef spartan::drake_robot_control::ExternalForceGuard ExternalForceGuard;
typedef spartan::drake_robot_control::ForceGuardContainer ForceGuardContainer;

std::shared_ptr<ForceGuardContainer>
ForceGuardContainerFromRosMsg(const robot_msgs::ForceGuard &msg,
                              const RigidBodyTreed &tree) {

  std::vector<std::shared_ptr<ForceGuard>> guards;

  // create TotalExternalTorqueGuard
  if (msg.joint_torque_external_l2_norm.size() > 0) {
    double threshold = msg.joint_torque_external_l2_norm[0];
    std::shared_ptr<TotalExternalTorqueGuard> guard =
        std::make_shared<TotalExternalTorqueGuard>(threshold);
    guards.push_back(guard);
  }

  for (int i = 0; i < msg.external_force_guards.size(); i++) {
    guards.push_back(
        ExternalForceGuardFromRosMsg(msg.external_force_guards[i], tree));
  }

  std::shared_ptr<ForceGuardContainer> guard_container;

  if (guards.size() > 0) {
    std::cout << boost::format(
                     "Created a ForceGuardContainer with %d guards\n") %
                     guards.size();
    guard_container = std::make_shared<ForceGuardContainer>();
    guard_container->AddGuards(guards);
  }

  return guard_container;
}

std::shared_ptr<ExternalForceGuard>
ExternalForceGuardFromRosMsg(const robot_msgs::ExternalForceGuard &msg,
                             const RigidBodyTreed &tree) {

  std::string body_frame = msg.body_frame;
  std::string expressed_in_frame = msg.force.header.frame_id;
  if (expressed_in_frame == "base") {
    expressed_in_frame = "world";
  }

  const auto &force_msg = msg.force.vector;
  Eigen::Vector3d force(force_msg.x, force_msg.y, force_msg.z);

  int idx_world = tree.FindBodyIndex("world");
  int idx_body = tree.FindBodyIndex(body_frame);
  int idx_expressed_in = tree.FindBodyIndex(expressed_in_frame);
  std::shared_ptr<ExternalForceGuard> guard =
      std::make_shared<ExternalForceGuard>(tree, idx_body, idx_world,
                                           idx_expressed_in, force);

  return guard;
}

std::unique_ptr<RobotPlanRunner>
RobotPlanRunner::GetInstance(ros::NodeHandle &nh,
                             const std::string &config_file_name) {
  YAML::Node config = YAML::LoadFile(config_file_name);
  if (!config["lcm_status_channel"] || !config["lcm_command_channel"] ||
      !config["lcm_plan_channel"] || !config["lcm_stop_channel"] ||
      !config["num_joints"] || !config["robot_ee_body_name"] ||
      !config["robot_urdf_path"] ||
      !config["joint_speed_limit_degree_per_sec"] ||
      !config["control_period_s"] ||
      !config["joint_limit_tolerance"] ||
      !config["joint_limits"]) {
    std::cerr << "Config file missing one or more fields." << std::endl;
    std::exit(1);
  }

  auto tree = std::make_unique<RigidBodyTreed>();
  std::string urdf_filename = config["robot_urdf_path"].as<std::string>();
  autoExpandEnvironmentVariables(urdf_filename);
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf_filename, multibody::joints::kFixed, tree.get());

  auto ptr = std::make_unique<RobotPlanRunner>(
      config["lcm_status_channel"].as<std::string>(),
      config["lcm_command_channel"].as<std::string>(),
      config["lcm_plan_channel"].as<std::string>(),
      config["lcm_stop_channel"].as<std::string>(),
      config["robot_ee_body_name"].as<std::string>(),
      config["num_joints"].as<int>(),
      config["joint_speed_limit_degree_per_sec"].as<double>(),
      config["control_period_s"].as<double>(), config, std::move(tree), nh);

  return std::move(ptr);
}

RobotPlanRunner::RobotPlanRunner(
    const std::string &lcm_status_channel,
    const std::string &lcm_command_channel, const std::string &lcm_plan_channel,
    const std::string &lcm_stop_channel, const std::string &robot_ee_body_name,
    int num_joints, double joint_speed_limit_deg_per_sec, double control_period,
    YAML::Node config,
    std::unique_ptr<const RigidBodyTreed> tree, ros::NodeHandle &nh)
    : kLcmStatusChannel_(lcm_status_channel),
      kLcmCommandChannel_(lcm_command_channel),
      kLcmPlanChannel_(lcm_plan_channel), kLcmStopChannel_(lcm_stop_channel),
      kRobotEeBodyName_(robot_ee_body_name), kNumJoints_(num_joints),
      kJointSpeedLimitDegPerSec_(joint_speed_limit_deg_per_sec),
      kControlPeriod_(control_period), config_(config), tree_(std::move(tree)), nh_(nh),
      plan_number_(0), tf_listener_(tf_buffer_),
      terminate_current_plan_flag_(false) {

  DRAKE_DEMAND(kNumJoints_ == tree_->get_num_positions());
  DRAKE_DEMAND(kNumJoints_ == tree_->get_num_actuators());
  this->LoadJointLimits();
  plan_number_ = 0;
  new_plan_ = nullptr;
  current_robot_state_.resize(kNumJoints_ * 2, 1);
  iiwa_status_position_command_.resize(kNumJoints_, 1);
  iiwa_status_torque_command_.resize(kNumJoints_, 1);
  last_position_command_.resize(kNumJoints_, 1);
  last_torque_command_.resize(kNumJoints_, 1);
  has_received_new_status_ = false;
  is_waiting_for_first_robot_status_message_ = true;

  // setup the ROS actions

  joint_trajectory_action_ = std::make_shared<
      actionlib::SimpleActionServer<robot_msgs::JointTrajectoryAction>>(
      nh_, "JointTrajectory",
      boost::bind(&RobotPlanRunner::ExecuteJointTrajectoryAction, this, _1),
      false);
  joint_trajectory_action_->start(); // start the ROS action

  cartesian_trajectory_action_ = std::make_shared<
      actionlib::SimpleActionServer<robot_msgs::CartesianTrajectoryAction>>(
      nh_, "CartesianTrajectory",
      boost::bind(&RobotPlanRunner::ExecuteCartesianTrajectoryAction, this, _1),
      false);
  cartesian_trajectory_action_->start(); // start the ROS action
  get_plan_number_action_ = std::make_shared<
      actionlib::SimpleActionServer<robot_msgs::GetPlanNumberAction>>(
      nh_, "GetPlanNumber",
      boost::bind(&RobotPlanRunner::GetPlanNumber, this, _1),
      false);
  get_plan_number_action_->start(); // start the ROS action
  

  // Set up the streaming plan management services and channels
  plan_end_server_ = std::make_shared<ros::ServiceServer>(
      nh_.advertiseService(
        "/plan_runner/stop_plan",
        &RobotPlanRunner::HandlePlanEndServiceCall, this));
  joint_space_streaming_plan_init_server_ = 
    std::make_shared<ros::ServiceServer>(
      nh_.advertiseService(
        "/plan_runner/init_joint_space_streaming",
        &RobotPlanRunner::HandleInitJointSpaceStreamingServiceCall, this));
  task_space_streaming_plan_init_server_ = 
    std::make_shared<ros::ServiceServer>(
      nh_.advertiseService(
        "/plan_runner/init_task_space_streaming",
        &RobotPlanRunner::HandleInitTaskSpaceStreamingServiceCall, this));
}

bool RobotPlanRunner::HandleInitJointSpaceStreamingServiceCall(
    robot_msgs::StartStreamingPlan::Request &req,
    robot_msgs::StartStreamingPlan::Response &res) {
  ROS_INFO("\n\n----JointSpaceStreaming Start------\n\n");
  ROS_INFO("Received Joint Space Streaming Plan");

  if (is_waiting_for_first_robot_status_message_) {
    std::cout << "Discarding plan, no status message received yet" << std::endl;
    res.status.status = res.status.ERROR;
    return false;
  }

  auto plan_local = std::make_shared<JointSpaceStreamingPlan>(tree_, nh_);

  std::cout << "started joint space streaming plan" << std::endl;

  // Add ForceGuards if specified
  if (req.force_guard.size() > 0) {

    const robot_msgs::ForceGuard force_guard_msg = req.force_guard[0];
    std::shared_ptr<ForceGuardContainer> guard_container =
        ForceGuardContainerFromRosMsg(force_guard_msg, *tree_);

    // if the shared_ptr is not null, it means there is at least one guard in
    // the guard container
    if (guard_container) {
      ROS_INFO("Adding ForceGuardContainer to plan");
      plan_local->set_guard_container(guard_container);
    }
  }

  QueueNewPlan(plan_local);

  res.status.status = res.status.RUNNING;
  ROS_INFO("\n\n------JointSpaceStreaming Successfully Started------\n\n");
  return true;
}

bool RobotPlanRunner::GetPlanNumber(const robot_msgs::GetPlanNumberGoal::ConstPtr &goal) {
  robot_msgs::GetPlanNumberResult result;
  result.plan_number = plan_number_;
  get_plan_number_action_->setSucceeded(result);
  return true;
}


bool RobotPlanRunner::HandleInitTaskSpaceStreamingServiceCall(
    robot_msgs::StartStreamingPlan::Request &req,
    robot_msgs::StartStreamingPlan::Response &res) {
  ROS_INFO("\n\n----TaskSpaceStreaming Start------\n\n");
  ROS_INFO("Received Task Space Streaming Plan");

  if (is_waiting_for_first_robot_status_message_) {
    std::cout << "Discarding plan, no status message received yet" << std::endl;
    res.status.status = res.status.ERROR;
    return false;
  }

  auto plan_local = std::make_shared<TaskSpaceStreamingPlan>(tree_, nh_);

  std::cout << "started task space streaming plan" << std::endl;

  // Add ForceGuards if specified
  if (req.force_guard.size() > 0) {

    const robot_msgs::ForceGuard force_guard_msg = req.force_guard[0];
    std::shared_ptr<ForceGuardContainer> guard_container =
        ForceGuardContainerFromRosMsg(force_guard_msg, *tree_);

    // if the shared_ptr is not null, it means there is at least one guard in
    // the guard container
    if (guard_container) {
      ROS_INFO("Adding ForceGuardContainer to plan");
      plan_local->set_guard_container(guard_container);
    }
  }

  QueueNewPlan(plan_local);

  res.status.status = res.status.RUNNING;
  res.plan_number = plan_local->plan_number_;
  ROS_INFO("\n\n------TaskSpaceStreaming Successfully Started------\n\n");
  return true;
}

bool RobotPlanRunner::HandlePlanEndServiceCall(
  std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  terminate_current_plan_flag_ = true;
  res.success = true;
  return true;
}

RobotPlanRunner::~RobotPlanRunner() {
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  if (subscriber_thread_.joinable()) {
    subscriber_thread_.join();
  }
  if (plan_constructor_thread_.joinable()) {
    plan_constructor_thread_.join();
  }
}

void RobotPlanRunner::Start() {
  publish_thread_ = std::thread(&RobotPlanRunner::PublishCommand, this);
  subscriber_thread_ = std::thread(&RobotPlanRunner::ReceiveRobotStatus, this);
  plan_constructor_thread_ =
      std::thread(&RobotPlanRunner::ConstructNewPlanFromLcm, this);
}

Eigen::VectorXd RobotPlanRunner::get_current_robot_state() {
  std::lock_guard<std::mutex> lock(robot_status_mutex_);
  return current_robot_state_;
}

Eigen::VectorXd RobotPlanRunner::get_current_robot_position() {
  std::lock_guard<std::mutex> lock(robot_status_mutex_);
  return current_robot_state_.head(kNumJoints_);
}

Eigen::VectorXd RobotPlanRunner::get_current_robot_velocity() {
  std::lock_guard<std::mutex> lock(robot_status_mutex_);
  return current_robot_state_.tail(kNumJoints_);
}

void RobotPlanRunner::LoadJointLimits(){
  joint_limit_tolerance_ = ToRadians(config_["joint_limit_tolerance"].as<double>());
  joint_limits_min_ = Eigen::VectorXd::Zero(kNumJoints_);
  joint_limits_max_ = Eigen::VectorXd::Zero(kNumJoints_);

  // iterate through all the joints
  for (int i = 0; i < kNumJoints_; i++){
    std::string joint_name = tree_->get_position_name(i);
    joint_limits_min_[i] = ToRadians(config_["joint_limits"][joint_name][0].as<double>()) +
        joint_limit_tolerance_;
    joint_limits_max_[i] = ToRadians(config_["joint_limits"][joint_name][1].as<double>()) -
        joint_limit_tolerance_;

//    std::cout << "\njoint_name = " << joint_name << std::endl;
//    std::cout << joint_limits_min_[i] << " " << joint_limits_max_[i] << std::endl;

  }

}

Eigen::VectorXd RobotPlanRunner::ApplyJointLimits(const Eigen::VectorXd & q_commanded){
  Eigen::VectorXd q_clamp = q_commanded.cwiseMax(joint_limits_min_);
  q_clamp = q_clamp.cwiseMin(joint_limits_max_);
  return q_clamp;
}



void RobotPlanRunner::ReceiveRobotStatus() {
  // lock mutex when printing so that the text is not mangled (by printing in
  // another thread).
  robot_status_mutex_.lock();
  std::cout << "Robot status receiver thread starting on thread "
            << std::this_thread::get_id() << std::endl;
  robot_status_mutex_.unlock();

  lcm::LCM receiver_lcm;
  receiver_lcm.subscribe(kLcmStatusChannel_, &RobotPlanRunner::HandleStatus,
                         this);

  while (true) {
    // Call lcm handle until at least one status message is
    // processed.
    while (0 == receiver_lcm.handleTimeout(10) ||
           is_waiting_for_first_robot_status_message_) {
      // TODO: Print something here so users know no LCM message has been
      // received.
    }
  }
}

void RobotPlanRunner::ConstructNewPlanFromLcm() {
  // lock mutex when printing so that the text is not mangled (by printing in
  // another thread).
  robot_status_mutex_.lock();
  std::cout << "New Plan constructor thread starting on thread "
            << std::this_thread::get_id() << std::endl;
  robot_status_mutex_.unlock();

  lcm::LCM constructor_lcm;
  constructor_lcm.subscribe(
      kLcmPlanChannel_, &RobotPlanRunner::HandleJointSpaceTrajectoryPlan, this);
  constructor_lcm.subscribe(kLcmStopChannel_, &RobotPlanRunner::HandleStop,
                            this);

  while (true) {
    // Call lcm handle until at least one status message is
    // processed.
    while (0 == constructor_lcm.handleTimeout(10)) {
      // TODO: Print something here so users know no LCM message has been
      // received.
    }
  }
}

void RobotPlanRunner::PublishCommand() {
  robot_status_mutex_.lock();
  std::cout << "Command publisher thread starting on thread "
            << std::this_thread::get_id() << std::endl;
  robot_status_mutex_.unlock();

  // Allocate and initialize stuff used in the loop.
  lcm::LCM publisher_lcm;
  std::shared_ptr<PlanBase> plan_local;

  const double max_dq_per_step =
      kJointSpeedLimitDegPerSec_ / 180 * M_PI * kControlPeriod_;

  lcmt_iiwa_status iiwa_status_local;
  iiwa_status_local.utime = -1;
  int64_t start_time_us = -1;
  int64_t cur_time_us = -1;
  double cur_plan_time_s = -1;
  bool has_published_command = false;
  // bool terminate_current_plan = false; // whether or not to

  lcmt_iiwa_command iiwa_command;
  iiwa_command.num_joints = kNumJoints_;
  iiwa_command.joint_position.resize(kNumJoints_, 0.);
  iiwa_command.num_torques = kNumJoints_;
  iiwa_command.joint_torque.resize(kNumJoints_, 0.);
  Eigen::VectorXd q_commanded(kNumJoints_), v_commanded(kNumJoints_),
      tau_commanded(kNumJoints_);

  Eigen::VectorXd prev_position_command(kNumJoints_);
  Eigen::VectorXd prev_torque_command(kNumJoints_);

  Eigen::VectorXd current_robot_state, cur_tau_external(kNumJoints_);

  while (true) {
    // Put the thread to sleep until a new iiwa_status message is received by
    // the subscriber thread.
    std::unique_lock<std::mutex> status_lock(robot_status_mutex_);
    cv_.wait(status_lock,
             std::bind(&RobotPlanRunner::has_received_new_status, this));
    has_received_new_status_ = false;

    // we should have the lock at this point

    // these should all be copies
    current_robot_state = current_robot_state_;
    iiwa_status_local = iiwa_status_; // this is a copy

    // Calling unlock is necessary because when cv_.wait() returns, this
    // thread acquires the mutex, preventing the receiver thread from
    // executing
    if (!has_published_command) {      
      prev_position_command = iiwa_status_position_command_;
      prev_torque_command = iiwa_status_torque_command_;
    }

    // update what you commanded
    last_position_command_ = prev_position_command;
    last_torque_command_ = prev_torque_command;

    status_lock.unlock();

    // see if there are any new plans
    robot_plan_mutex_.lock();
    if (terminate_current_plan_flag_.load() == true) {
      std::cout << "Terminating current plan" << std::endl;
      plan_local->set_plan_status(PlanStatus::STOPPED_BY_EXTERNAL_TRIGGER);
      plan_local->SetPlanFinished();
      terminate_current_plan_flag_.store(false);
      plan_local.reset();
    } else if (new_plan_) {
      std::cout << "New plan swapped into publisher thread" << std::endl;
      plan_local = new_plan_;
      new_plan_.reset();
    }
    robot_plan_mutex_.unlock();

    cur_time_us = iiwa_status_local.utime;
    for (int i = 0; i < kNumJoints_; i++) {
      cur_tau_external[i] = iiwa_status_local.joint_torque_external[i];
    }

    if (!plan_local) {
      std::cout << "plan_local == nullptr, holding current position..."
                << std::endl;

      // use the last commanded robot position
      plan_local = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(
          tree_, prev_position_command);

      // update the plan number manually since we aren't using the
      // QueueNewPlan function
      plan_local->plan_number_ = plan_number_++;
    }

    // special logic if the plan is new, i.e. not yet in state RUNNING
    if (plan_local->get_plan_status() == PlanStatus::NOT_STARTED) {
      std::cout << "\nStarting plan No. " << plan_number_ << std::endl;

      plan_local->SetCurrentCommand(prev_position_command, prev_torque_command);
      start_time_us = iiwa_status_local.utime;
    }

    cur_plan_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;
    plan_local->Step(current_robot_state, cur_tau_external, cur_plan_time_s,
                     &q_commanded, &v_commanded, &tau_commanded);

    // apply joint limits
    q_commanded = this->ApplyJointLimits(q_commanded);

    plan_local->SetCurrentCommand(q_commanded, tau_commanded);

    // Discard current plan if commanded position is "too far away" from
    // the previous commanded position, i.e. the commanded joint trajectory is
    // not sufficiently smooth.
    Eigen::VectorXd dq_cmd = q_commanded - prev_position_command;
    bool unsafe_command = false;
    for (int i = 0; i < kNumJoints_; i++) {

      if ((std::abs(dq_cmd[i]) > max_dq_per_step)) {
        std::cout << "Commanded joint position is too jerky, discarding plan..."
                  << std::endl;
        std::cout << "dq_cmd limit: " << max_dq_per_step << std::endl;
        std::cout << "Commanded dq_cmd[" << i << "]: " << dq_cmd[i]
                  << std::endl;
        unsafe_command = true;
      }

      if (std::abs(tau_commanded[i]) > 1.0) {
        std::cout << "Non-zero torque command detected, stopping" << std::endl;
        unsafe_command = true;
      }

      if (std::isnan(q_commanded[i])) {
        std::cout << "\nCommand is nan, discarding" << std::endl;
        std::cout << "\nposition_command:\n" << q_commanded << std::endl;
        std::cout << "\nvelocity_command:\n" << v_commanded << std::endl;
        std::cout << "\ntorque_command:\n" << tau_commanded << std::endl;
        std::cout << "Current_robot_state:\n"
                  << current_robot_state << std::endl;
        std::cout << "\ncur_plan_time: " << cur_plan_time_s << std::endl;
        unsafe_command = true;

        std::cout << "plan_local->get_plan_status():"
                  << plan_local->get_plan_status() << std::endl;
        std::cout << "plan_local->is_finished_:" << plan_local->is_finished_
                  << std::endl;
      }

      if (unsafe_command) {
        std::cout << "detected unsafe command\n";
        std::cout << "\nposition_command:\n" << q_commanded << std::endl;
        std::cout << "\ntorque_command:\n" << tau_commanded << std::endl;

        std::cout << "sending previous position command instead" << std::endl;
        std::cout << "setting torque command to zero" << std::endl;
        q_commanded = prev_position_command;
        tau_commanded = Eigen::VectorXd::Zero(kNumJoints_);

        std::cout << "safe commands" << std::endl;
        std::cout << "\nposition_command:\n" << q_commanded << std::endl;
        std::cout << "\ntorque_command:\n" << tau_commanded << std::endl;

        plan_local->set_plan_status(PlanStatus::STOPPED_BY_SAFETY_CHECK);
        plan_local->SetPlanFinished();
        std::cout << "set current plan to finished\n";
        plan_local.reset();
        std::cout << "reset plan_local pointer\n";
        break;
      }
    }

    // construct and publish iiwa_command
    iiwa_command.utime = iiwa_status_local.utime;
    for (int i = 0; i < kNumJoints_; i++) {
      iiwa_command.joint_position[i] = q_commanded(i);
      iiwa_command.joint_torque[i] = tau_commanded(i);
    }
    publisher_lcm.publish(kLcmCommandChannel_, &iiwa_command);
    has_published_command = true;
    prev_position_command = q_commanded;
    prev_torque_command = tau_commanded;
  }
}

void RobotPlanRunner::MoveToJointPosition(
    const Eigen::Ref<const Eigen::VectorXd> q_final, double duration) {
  Eigen::VectorXd q0 = get_current_robot_position();
  std::vector<double> times{0, duration};
  std::vector<Eigen::MatrixXd> knots;
  knots.push_back(q0);
  knots.push_back(q_final);

  std::shared_ptr<PlanBase> plan = std::make_shared<JointSpaceTrajectoryPlan>(
      tree_, PPType::FirstOrderHold(times, knots));
  QueueNewPlan(plan);
}

// void RobotPlanRunner::MoveRelativeToCurrentEeCartesianPosition(
//    const Eigen::Ref<const Eigen::Vector3d> delta_xyz_ee,
//    const math::RotationMatrixd &R_WE_ref, double duration) {
//  Eigen::Isometry3d T_ee;
//  Eigen::Vector3d rpy;
//  GetEePoseInWorldFrame(&T_ee, &rpy);
//  const math::RotationMatrixd R_WE(T_ee.linear());
//
//  Eigen::Vector3d xyz_ee_start = T_ee.translation();
//  Eigen::Vector3d xyz_ee_final = xyz_ee_start + delta_xyz_ee;
//
//  std::vector<double> times{0, duration};
//  std::vector<Eigen::MatrixXd> knots;
//  knots.push_back(xyz_ee_start);
//  knots.push_back(xyz_ee_final);
//
//  std::shared_ptr<PlanBase> plan;
//
//  if (!R_WE.IsNearlyEqualTo(R_WE_ref, 0.5)) {
//    std::cout
//        << "Difference between current and reference orietation is too large,
//        "
//           "keep current orietation..."
//        << std::endl;
//    plan = std::make_shared<EndEffectorOriginTrajectoryPlan>(
//        tree_, PPType::FirstOrderHold(times, knots), R_WE,
//        kRobotEeBodyName_);
//  } else {
//    plan = std::make_unique<EndEffectorOriginTrajectoryPlan>(
//        tree_, PPType::FirstOrderHold(times, knots), R_WE_ref,
//        kRobotEeBodyName_);
//  }
//  QueueNewPlan(plan);
//}

void RobotPlanRunner::HandleStatus(const lcm::ReceiveBuffer *,
                                   const std::string &,
                                   const lcmt_iiwa_status *status) {
  std::unique_lock<std::mutex> lock(robot_status_mutex_);
  iiwa_status_ = *status;
  has_received_new_status_ = true;

  for (int i = 0; i < kNumJoints_; i++) {
    current_robot_state_[i] = iiwa_status_.joint_position_measured[i];
    current_robot_state_[i + kNumJoints_] =
        iiwa_status_.joint_velocity_estimated[i];
    iiwa_status_position_command_[i] = iiwa_status_.joint_position_commanded[i];
    iiwa_status_torque_command_[i] = iiwa_status_.joint_torque_commanded[i];
  }
  lock.unlock();


  if (is_waiting_for_first_robot_status_message_){
    last_position_command_ = iiwa_status_position_command_;
    last_torque_command_ = iiwa_status_torque_command_;
  }
  is_waiting_for_first_robot_status_message_ = false;
  cv_.notify_all();
}

void RobotPlanRunner::HandleJointSpaceTrajectoryPlan(
    const lcm::ReceiveBuffer *, const std::string &,
    const robotlocomotion::robot_plan_t *tape) {
  // Most of the code in this function is copied from
  // drake/examples/kuka_iiwa_arm/kuka_plan_runner.cc
  std::cout << "New joint space trajectory plan received." << std::endl;
  if (is_waiting_for_first_robot_status_message_) {
    std::cout << "Discarding plan, no status message received yet" << std::endl;
    return;
  } else if (tape->num_states < 2) {
    std::cout << "Discarding plan, Not enough knot points." << std::endl;
    return;
  }

  robot_status_mutex_.lock();
  auto iiwa_status_local = iiwa_status_;
  auto last_position_command_local = this->last_position_command_;
  auto last_torque_command_local = this->last_torque_command_;
  robot_status_mutex_.unlock();

  std::vector<Eigen::MatrixXd> knots(tape->num_states,
                                     Eigen::MatrixXd::Zero(kNumJoints_, 1));
  std::map<std::string, int> name_to_idx =
      tree_->computePositionNameToIndexMap();
  for (int i = 0; i < tape->num_states; ++i) {
    const auto &state = tape->plan[i];
    for (int j = 0; j < state.num_joints; ++j) {
      if (name_to_idx.count(state.joint_name[j]) == 0) {
        continue;
      }
      // Treat the matrix at knots[i] as a column vector.
      if (i == 0) {
        // Always start moving from the position which we're
        // currently commanding.
        knots[0](name_to_idx[state.joint_name[j]], 0) =
            last_position_command_local[j];
      } else {
        knots[i](name_to_idx[state.joint_name[j]], 0) = state.joint_position[j];
      }
    }
  }

  std::vector<double> input_time;
  for (int k = 0; k < static_cast<int>(tape->plan.size()); ++k) {
    input_time.push_back(tape->plan[k].utime / 1e6);
  }

  const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints_, 1);
  auto plan_new_local = std::make_shared<JointSpaceTrajectoryPlan>(
      tree_, PPType::Cubic(input_time, knots, knot_dot, knot_dot));

  QueueNewPlan(plan_new_local);
}

void RobotPlanRunner::ExecuteJointTrajectoryAction(
    const robot_msgs::JointTrajectoryGoal::ConstPtr &goal) {

  ROS_INFO("\n\n----JointTrajectoryAction Start------\n\n");
  ROS_INFO("Received Joint Space Trajectory Plan");
  // robot_msgs::JointTrajectoryResult result;

  int num_knot_points = goal->trajectory.points.size();
  const trajectory_msgs::JointTrajectory &trajectory = goal->trajectory;

  if (is_waiting_for_first_robot_status_message_) {
    std::cout << "Discarding plan, no status message received yet" << std::endl;
    return;
  } else if (num_knot_points < 2) {
    std::cout << "Discarding plan, Not enough knot points." << std::endl;
    return;
  }

  robot_status_mutex_.lock();
  auto iiwa_status_local = iiwa_status_;
  auto last_position_command_local = this->last_position_command_;
  auto last_torque_command_local = this->last_torque_command_;
  robot_status_mutex_.unlock();

  std::vector<Eigen::MatrixXd> knots(num_knot_points,
                                     Eigen::MatrixXd::Zero(kNumJoints_, 1));
  std::map<std::string, int> name_to_idx =
      tree_->computePositionNameToIndexMap();

  std::vector<double> input_time;
  for (int i = 0; i < num_knot_points; ++i) {
    const trajectory_msgs::JointTrajectoryPoint &traj_point =
        trajectory.points[i];
    for (int j = 0; j < trajectory.joint_names.size(); ++j) {
      std::string joint_name = trajectory.joint_names[j];
      if (name_to_idx.count(joint_name) == 0) {
        continue;
      }

      int joint_idx = name_to_idx[joint_name];
      // Treat the matrix at knots[i] as a column vector.
      if (i == 0) {
        // Always start moving from the position which we're
        // currently commanding.
        knots[0](joint_idx, 0) = last_position_command_local[j];
      } else {
        knots[i](joint_idx, 0) = traj_point.positions[j];
      }
    }

    input_time.push_back(traj_point.time_from_start.toSec());
  }

  std::cout << "plan duration in seconds: " << input_time.back() << std::endl;

  const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints_, 1);
  auto plan_local = std::make_shared<JointSpaceTrajectoryPlan>(
      tree_, PPType::Cubic(input_time, knots, knot_dot, knot_dot));

  // Add ForceGuards if specified
  if (goal->force_guard.size() > 0) {

    const robot_msgs::ForceGuard force_guard_msg = goal->force_guard[0];
    std::shared_ptr<ForceGuardContainer> guard_container =
        ForceGuardContainerFromRosMsg(force_guard_msg, *tree_);

    // if the shared_ptr is not null, it means there is at least one guard in
    // the guard container
    if (guard_container) {
      ROS_INFO("Adding ForceGuardContainer to plan");
      plan_local->set_guard_container(guard_container);
    }
  }

  QueueNewPlan(plan_local);

  // // now wait for the plan to finish
  ROS_INFO("Waiting for plan to finish");
  plan_local->WaitForPlanToFinish();

  // // set the result of the action
  // ROS_INFO("setting ROS action to succeeded state");
  robot_msgs::JointTrajectoryResult result;
  plan_local->GetPlanStatusMsg(result.status);

  ROS_INFO("setting action result");
  joint_trajectory_action_->setSucceeded(result);

  ROS_INFO("\n\n------JointTrajectoryAction Finished------\n\n");
}

void RobotPlanRunner::ExecuteCartesianTrajectoryAction(
    const robot_msgs::CartesianTrajectoryGoal::ConstPtr &goal) {

  ROS_INFO("\n\n-------CartesianTrajectoryAction Start--------\n\n");
  const robot_msgs::CartesianTrajectory &traj = goal->trajectory;

  int num_knot_points = traj.xyz_points.size();

  if (is_waiting_for_first_robot_status_message_) {
    std::cout << "Discarding plan, no status message received yet" << std::endl;
    return;
  } else if (num_knot_points < 2) {
    std::cout << "Discarding plan, Not enough knot points." << std::endl;
    return;
  }

  robot_status_mutex_.lock();
  auto iiwa_status_local = iiwa_status_;
  auto last_position_command_local = this->last_position_command_;
  auto last_torque_command_local = this->last_torque_command_;
  robot_status_mutex_.unlock();

  // extract the xyz trajectory

  // frame in which xyz_traj is expressed.
  // It can either be world (denoted by base)
  // or it can be a local frame on the robot
  std::string xyz_traj_frame_id = traj.xyz_points[0].header.frame_id;
  std::string base_frame_id = "base";
  //  std::cout << "xyz_traj_frame_id: " << xyz_traj_frame_id << std::endl;
  geometry_msgs::TransformStamped transform_tf;

  try {
    transform_tf = tf_buffer_.lookupTransform(base_frame_id, xyz_traj_frame_id,
                                              ros::Time(0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // convert to Eigen transform
  Eigen::Affine3d T_local_to_world =
      spartan::drake_robot_control::utils::transformToEigen(transform_tf);

  //
  // lookup current position of ee_frame_id
  geometry_msgs::TransformStamped ee_frame_tf;
  try {
    ee_frame_tf = tf_buffer_.lookupTransform(base_frame_id, traj.ee_frame_id,
                                             ros::Time(0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  Eigen::Affine3d T_ee_to_world =
      spartan::drake_robot_control::utils::transformToEigen(ee_frame_tf);

  std::vector<Eigen::MatrixXd> knots(num_knot_points,
                                     Eigen::MatrixXd::Zero(3, 1));
  std::vector<double> input_time;

  // Local to world transformation

  // figure out what frame the points are expressed in
  for (int i = 0; i < num_knot_points; i++) {

    // replace first knot point by current position of ee_frame
    Eigen::Vector3d xyz_pos_in_world;
    if (i == 0) {
      xyz_pos_in_world = T_ee_to_world.translation();
    } else {

      const geometry_msgs::PointStamped &xyz_point = traj.xyz_points[i];
      Eigen::Vector3d xyz_pos_local(xyz_point.point.x, xyz_point.point.y,
                                    xyz_point.point.z);
      xyz_pos_in_world = T_local_to_world * xyz_pos_local;
    }

    knots[i] = xyz_pos_in_world;
    input_time.push_back(traj.time_from_start[i].toSec());
  }

  drake::math::RotationMatrixd R_ee_to_world_initial(T_ee_to_world.linear());
  drake::math::RotationMatrixd R_ee_to_world_final;

  if (traj.quaternions.size() > 0) {
    ROS_INFO("Orientation passed in, interpolating with slerp");
    const geometry_msgs::Quaternion &quat_msg = traj.quaternions[0];
    Eigen::Quaterniond quat_WE_final(quat_msg.w, quat_msg.x, quat_msg.y,
                                     quat_msg.z);
    R_ee_to_world_final = drake::math::RotationMatrixd(quat_WE_final);
  } else {
    ROS_INFO("No orientation passed in, using current");
    R_ee_to_world_final = R_ee_to_world_initial;
  }

  // figure out the gains
  Eigen::Vector3d kp_rotation;
  Eigen::Vector3d kp_translation;
  if (goal->gains.size() > 0) {

    kp_rotation =
        Eigen::Vector3d(goal->gains[0].rotation.x, goal->gains[0].rotation.y,
                        goal->gains[0].rotation.z);

    kp_translation = Eigen::Vector3d(goal->gains[0].translation.x,
                                     goal->gains[0].translation.y,
                                     goal->gains[0].translation.z);
  } else {
    kp_rotation =
        Eigen::Vector3d(this->config_["task_space_plan"]["kp_rotation"]
                            .as<std::vector<double>>()
                            .data());
    kp_translation =
        Eigen::Vector3d(this->config_["task_space_plan"]["kp_translation"]
                            .as<std::vector<double>>()
                            .data());
  }

  const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(3, 1);
  auto plan_local = std::make_shared<EndEffectorOriginTrajectoryPlan>(
      tree_, PPType::Cubic(input_time, knots, knot_dot, knot_dot),
      R_ee_to_world_initial, R_ee_to_world_final, kp_rotation, kp_translation,
      traj.ee_frame_id);

  // Add ForceGuards if specified
  if (goal->force_guard.size() > 0) {

    const robot_msgs::ForceGuard force_guard_msg = goal->force_guard[0];
    std::shared_ptr<ForceGuardContainer> guard_container =
        ForceGuardContainerFromRosMsg(force_guard_msg, *tree_);

    // if the shared_ptr is not null, it means there is at least one guard in
    // the guard container
    if (guard_container) {
      ROS_INFO("Adding ForceGuardContainer to plan");
      plan_local->set_guard_container(guard_container);
    }
  }

  QueueNewPlan(plan_local);
  ROS_INFO("Waiting for plan to finish");

  PlanStatus plan_status = plan_local->WaitForPlanToFinish();

  robot_msgs::CartesianTrajectoryResult result;
  plan_local->GetPlanStatusMsg(result.status);
  ROS_INFO("setting action result");
  cartesian_trajectory_action_->setSucceeded(result);
  ROS_INFO("\n\n------CartesianTrajectoryAction Finished------\n\n");
}

void RobotPlanRunner::GetBodyPoseInWorldFrame(const RigidBody<double> &body,
                                              Eigen::Isometry3d *const T,
                                              Eigen::Vector3d *const rpy) {
  Eigen::VectorXd q = get_current_robot_position();
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  cache.initialize(q);
  tree_->doKinematics(cache);

  *T = tree_->CalcBodyPoseInWorldFrame(cache, body);
  *rpy = tree_->relativeRollPitchYaw(cache, body.get_body_index(), 0);
}

} // namespace robot_plan_runner
} // namespace drake
