#include <drake_robot_control/plan_runner.h>
#include <yaml-cpp/yaml.h>

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace drake {
namespace robot_plan_runner {

std::unique_ptr<RobotPlanRunner>
RobotPlanRunner::GetInstance(ros::NodeHandle& nh, const std::string &config_file_name) {
  YAML::Node config = YAML::LoadFile(config_file_name);
  if (!config["lcm_status_channel"] || !config["lcm_command_channel"] ||
      !config["lcm_plan_channel"] || !config["lcm_stop_channel"] ||
      !config["num_joints"] || !config["robot_ee_body_name"] ||
      !config["robot_urdf_path"] ||
      !config["joint_speed_limit_degree_per_sec"] ||
      !config["control_period_s"]) {
    std::cerr << "Config file missing one or more fields." << std::endl;
    std::exit(1);
  }

  auto tree = std::make_unique<RigidBodyTreed>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(config["robot_urdf_path"].as<std::string>().c_str()),
      multibody::joints::kFixed, tree.get());

  auto ptr = std::make_unique<RobotPlanRunner>(
      config["lcm_status_channel"].as<std::string>(),
      config["lcm_command_channel"].as<std::string>(),
      config["lcm_plan_channel"].as<std::string>(),
      config["lcm_stop_channel"].as<std::string>(),
      config["robot_ee_body_name"].as<std::string>(),
      config["num_joints"].as<int>(),
      config["joint_speed_limit_degree_per_sec"].as<double>(),
      config["control_period_s"].as<double>(), std::move(tree), nh);

  return std::move(ptr);
}

RobotPlanRunner::RobotPlanRunner(
    const std::string &lcm_status_channel,
    const std::string &lcm_command_channel, const std::string &lcm_plan_channel,
    const std::string &lcm_stop_channel, const std::string &robot_ee_body_name,
    int num_joints, double joint_speed_limit_deg_per_sec, double control_period,
    std::unique_ptr<const RigidBodyTreed> tree,
    ros::NodeHandle& nh)
    : kLcmStatusChannel_(lcm_status_channel),
      kLcmCommandChannel_(lcm_command_channel),
      kLcmPlanChannel_(lcm_plan_channel), kLcmStopChannel_(lcm_stop_channel),
      kRobotEeBodyName_(robot_ee_body_name), kNumJoints_(num_joints),
      kJointSpeedLimitDegPerSec_(joint_speed_limit_deg_per_sec),
      kControlPeriod_(control_period), tree_(std::move(tree)),
      nh_(nh),
      joint_trajectory_action_(nh_, "JointTrajectory", boost::bind(&RobotPlanRunner::ExecuteJointTrajectoryAction, this, _1), false) {

  DRAKE_DEMAND(kNumJoints_ == tree_->get_num_positions());
  DRAKE_DEMAND(kNumJoints_ == tree_->get_num_actuators());
  plan_number_ = 0;
  new_plan_ = nullptr;
  current_robot_state_.resize(kNumJoints_ * 2, 1);
  has_received_new_status_ = false;
  is_waiting_for_first_robot_status_message_ = true;
  joint_trajectory_action_.start(); // start the ROS action
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

  std::cout << "starting ros node" << std::endl;
  ros::spin(); // start the ROS node, this spins on the main thread
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
  std::unique_ptr<PlanBase> plan_local;

  const double max_dq_per_step =
      kJointSpeedLimitDegPerSec_ / 180 * M_PI * kControlPeriod_;

  lcmt_iiwa_status iiwa_status_local;
  iiwa_status_local.utime = -1;
  int64_t start_time_us = -1;
  int64_t cur_time_us = -1;
  double cur_plan_time_s = -1;
  bool has_new_plan = false;
  bool is_terminated = false;

  lcmt_iiwa_command iiwa_command;
  iiwa_command.num_joints = kNumJoints_;
  iiwa_command.joint_position.resize(kNumJoints_, 0.);
  iiwa_command.num_torques = kNumJoints_;
  iiwa_command.joint_torque.resize(kNumJoints_, 0.);
  Eigen::VectorXd q_commanded(kNumJoints_), v_commanded(kNumJoints_),
      tau_commanded(kNumJoints_);
  Eigen::VectorXd current_robot_state, cur_tau_external(kNumJoints_);
  Eigen::VectorXd q_commanded_prev(kNumJoints_);

  while (true) {
    // Put the thread to sleep until a new iiwa_status message is received by
    // the subscriber thread.
    std::unique_lock<std::mutex> status_lock(robot_status_mutex_);
    cv_.wait(status_lock,
             std::bind(&RobotPlanRunner::has_received_new_status, this));
    has_received_new_status_ = false;
    current_robot_state = current_robot_state_;
    iiwa_status_local = iiwa_status_;

    // Calling unlock is necessary because when cv_.wait() returns, this
    // thread acquires the mutex, preventing the receiver thread from
    // executing.
    status_lock.unlock();

    // Lock plan_mutex when checking for new plans.
    robot_plan_mutex_.lock();
    if (is_plan_terminated_externally_) {
      is_plan_terminated_externally_ = false;
      is_terminated = true;
    } else if (new_plan_) {
      plan_local = std::move(new_plan_);
      has_new_plan = true;
    }
    robot_plan_mutex_.unlock();

    cur_time_us = iiwa_status_local.utime;
    for (int i = 0; i < kNumJoints_; i++) {
      cur_tau_external[i] = iiwa_status_local.joint_torque_external[i];
    }

    // Make local changes after checking for new plans.
    if (is_terminated) {
      is_terminated = false;
      plan_local.reset();
    } else if (has_new_plan) {
      has_new_plan = false;
      plan_number_++;
      start_time_us = iiwa_status_local.utime;
      std::cout << "\nStarting plan No. " << plan_number_ << std::endl;
    }

    if (!plan_local) {
      std::cout << "plan_local == nullptr, holding current position..."
                << std::endl;
      plan_local = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(
          tree_, current_robot_state.head(kNumJoints_));
    }

    cur_plan_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;
    plan_local->Step(current_robot_state, cur_tau_external, cur_plan_time_s,
                     &q_commanded, &v_commanded, &tau_commanded);

    for (int j = 0; j < kNumJoints_; j++) {
      q_commanded_prev[j] = iiwa_status_local.joint_position_commanded[j];
    }

    // Discard current plan if commanded position is "too far away" from
    // the previous commanded position, i.e. the commanded joint trajectory is
    // not sufficiently smooth.
    Eigen::VectorXd dq_cmd = q_commanded - q_commanded_prev;

    for (int i = 0; i < kNumJoints_; i++) {
      if (std::abs(dq_cmd[i]) > max_dq_per_step) {
        std::cout << "Commanded joint position is too jerky, discarding plan..."
                  << std::endl;
        std::cout << "dq_cmd limit: " << max_dq_per_step << std::endl;
        std::cout << "Commanded dq_cmd[" << i << "]: " << dq_cmd[i]
                  << std::endl;
        plan_local.reset();
        // Publish commanded position from previous control tick.
        q_commanded = q_commanded_prev;
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
  }
}

void RobotPlanRunner::MoveToJointPosition(
    const Eigen::Ref<const Eigen::VectorXd> q_final, double duration) {
  Eigen::VectorXd q0 = get_current_robot_position();
  std::vector<double> times{0, duration};
  std::vector<Eigen::MatrixXd> knots;
  knots.push_back(q0);
  knots.push_back(q_final);

  std::unique_ptr<PlanBase> plan = std::make_unique<JointSpaceTrajectoryPlan>(
      tree_, PPType::FirstOrderHold(times, knots));
  QueueNewPlan(std::move(plan));
}

void RobotPlanRunner::MoveRelativeToCurrentEeCartesianPosition(
    const Eigen::Ref<const Eigen::Vector3d> delta_x_ee, double duration) {
  auto T_ee = get_body_pose_in_world_frame(*tree_->FindBody(kRobotEeBodyName_));
  Eigen::Vector3d x_ee_start = T_ee.translation();
  Eigen::Vector3d x_ee_final = x_ee_start + delta_x_ee;
  std::vector<double> times{0, duration};
  std::vector<Eigen::MatrixXd> knots;
  knots.push_back(x_ee_start);
  knots.push_back(x_ee_final);

  std::unique_ptr<PlanBase> plan =
      std::make_unique<EndEffectorOriginTrajectoryPlan>(
          tree_, PPType::FirstOrderHold(times, knots));
  QueueNewPlan(std::move(plan));
}

void RobotPlanRunner::HandleStatus(const lcm::ReceiveBuffer *,
                                   const std::string &,
                                   const lcmt_iiwa_status *status) {
  std::unique_lock<std::mutex> lock(robot_status_mutex_);
  iiwa_status_ = *status;
  has_received_new_status_ = true;
  is_waiting_for_first_robot_status_message_ = false;
  for (int i = 0; i < kNumJoints_; i++) {
    current_robot_state_[i] = iiwa_status_.joint_position_measured[i];
    current_robot_state_[i + kNumJoints_] =
        iiwa_status_.joint_velocity_estimated[i];
  }
  lock.unlock();
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
            iiwa_status_local.joint_position_commanded[j];
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
  auto plan_new_local = std::make_unique<JointSpaceTrajectoryPlan>(
      tree_, PPType::Cubic(input_time, knots, knot_dot, knot_dot));

  QueueNewPlan(std::move(plan_new_local));
}

void RobotPlanRunner::ExecuteJointTrajectoryAction(const robot_msgs::JointTrajectoryGoal::ConstPtr &goal){

  ROS_INFO("Received Joint Space Trajectory Plan");
  robot_msgs::JointTrajectoryResult result;
  

  int num_knot_points = goal->trajectory.points.size();
  const trajectory_msgs::JointTrajectory & trajectory = goal->trajectory;



  ROS_INFO("Received Joint Space Trajectory Plan");
  if (is_waiting_for_first_robot_status_message_) {
    std::cout << "Discarding plan, no status message received yet" << std::endl;
    return;                                                                                            
  } else if (num_knot_points < 2) {
    std::cout << "Discarding plan, Not enough knot points." << std::endl;
    return;
  }

  robot_status_mutex_.lock();
  auto iiwa_status_local = iiwa_status_;
  robot_status_mutex_.unlock();



  std::vector<Eigen::MatrixXd> knots(num_knot_points,
                                     Eigen::MatrixXd::Zero(kNumJoints_, 1));
  std::map<std::string, int> name_to_idx =
      tree_->computePositionNameToIndexMap();

  std::vector<double> input_time;
  for (int i = 0; i < num_knot_points; ++i) {
    const trajectory_msgs::JointTrajectoryPoint & traj_point = trajectory.points[i];
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
        knots[0](joint_idx, 0) =
            iiwa_status_local.joint_position_commanded[j];
      } else {
        knots[i](joint_idx, 0) = traj_point.positions[j];
      }
    }

    input_time.push_back(traj_point.time_from_start.toSec());
  }

  std::cout << "plan duration in seconds: " << input_time.back() << std::endl;

  const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints_, 1);
  auto plan_new_local = std::make_unique<JointSpaceTrajectoryPlan>(
      tree_, PPType::Cubic(input_time, knots, knot_dot, knot_dot));

  QueueNewPlan(std::move(plan_new_local));

  result.status.status = result.status.FINISHED_NORMALLY;
  joint_trajectory_action_.setSucceeded(result);
}

Eigen::Isometry3d
RobotPlanRunner::get_body_pose_in_world_frame(const RigidBody<double> &body) {
  Eigen::VectorXd q = get_current_robot_position();
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  cache.initialize(q);
  tree_->doKinematics(cache);
  return tree_->CalcBodyPoseInWorldFrame(cache, body);
}

} // namespace robot_plan_runner
} // namespace drake
