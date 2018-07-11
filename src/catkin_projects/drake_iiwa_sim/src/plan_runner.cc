#include <robot_plan_runner/plan_runner.h>

namespace drake {
namespace robot_plan_runner {

RobotPlanRunner::RobotPlanRunner() : plan_number_(0), new_plan_(nullptr) {
  tree_ = std::make_unique<RigidBodyTreed>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_no_collision.urdf"),
      multibody::joints::kFixed, tree_.get());
  DRAKE_DEMAND(kNumJoints == tree_->get_num_positions());
  DRAKE_DEMAND(kNumJoints == tree_->get_num_actuators());
  current_robot_state_.resize(kNumJoints * 2, 1);
  has_received_new_status_ = false;
  is_waiting_for_first_robot_status_message_ = true;
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
  return current_robot_state_.head(kNumJoints);
}

Eigen::VectorXd RobotPlanRunner::get_current_robot_velocity() {
  std::lock_guard<std::mutex> lock(robot_status_mutex_);
  return current_robot_state_.tail(kNumJoints);
}

void RobotPlanRunner::ReceiveRobotStatus() {
  // lock mutex when printing so that the text is not mangled (by printing in
  // another thread).
  robot_status_mutex_.lock();
  std::cout << "Robot status receiver thread starting on thread "
            << std::this_thread::get_id() << std::endl;
  robot_status_mutex_.unlock();

  lcm::LCM receiver_lcm;
  receiver_lcm.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus,
                         this);

  while (true) {
    // Call lcm handle until at least one status message is
    // processed.
    while (0 == receiver_lcm.handleTimeout(10) ||
           is_waiting_for_first_robot_status_message_) {
      // Print something here so users know no LCM message has been received.
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
  constructor_lcm.subscribe(kLcmPlanChannel, &RobotPlanRunner::HandlePlan,
                            this);
  constructor_lcm.subscribe(kLcmStopChannel, &RobotPlanRunner::HandleStop,
                            this);

  while (true) {
    // Call lcm handle until at least one status message is
    // processed.
    while (0 == constructor_lcm.handleTimeout(10)) {
      // Print something here so users know no LCM message has been received.
    }
  }
}

void RobotPlanRunner::PublishCommand() {
  robot_status_mutex_.lock();
  std::cout << "Command publisher thread starting on thread "
            << std::this_thread::get_id() << std::endl;
  robot_status_mutex_.unlock();

  lcm::LCM publisher_lcm;
  std::unique_ptr<PlanBase> plan_local;

  lcmt_iiwa_status iiwa_status;
  iiwa_status.utime = -1;
  int64_t start_time_us = -1;
  int64_t cur_time_us = -1;
  double cur_plan_time_s = -1;
  bool has_new_plan = false;
  bool is_terminated = false;

  // Allocate and initialize stuff used in the loop.
  lcmt_iiwa_command iiwa_command;
  iiwa_command.num_joints = kNumJoints;
  iiwa_command.joint_position.resize(kNumJoints, 0.);
  iiwa_command.num_torques = 0;
  iiwa_command.joint_torque.resize(kNumJoints, 0.);
  Eigen::VectorXd q_commanded(kNumJoints), v_commanded(kNumJoints);
  Eigen::VectorXd current_robot_state;

  while (true) {
    // Put the thread to sleep until a new iiwa_status message is received by
    // the subscriber thread.
    std::unique_lock<std::mutex> status_lock(robot_status_mutex_);
    cv_.wait(status_lock,
             std::bind(&RobotPlanRunner::has_received_new_status, this));
    has_received_new_status_ = false;
    current_robot_state = current_robot_state_;
    iiwa_status = iiwa_status_;

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

    // Make local changes after receiving a new plan.
    cur_time_us = iiwa_status.utime;
    if (is_terminated) {
      is_terminated = false;
      plan_local.reset();
    } else if (has_new_plan) {
      has_new_plan = false;
      plan_number_++;
      start_time_us = iiwa_status.utime;
      std::cout << "Starting plan No. " << plan_number_ << std::endl;
    }

    if (!plan_local) {
      std::cout << "plan_local == nullptr, holding current position..."
                << std::endl;
      plan_local = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(
          tree_, current_robot_state.head(kNumJoints));
    }

    cur_plan_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;
    plan_local->Step(current_robot_state, cur_plan_time_s, &q_commanded,
                     &v_commanded);

    // Discard current plan if commanded position is "too far away" from
    // current position.
    Eigen::VectorXd dq = q_commanded - current_robot_state.head(kNumJoints);
    for (int i = 0; i < kNumJoints; i++) {
      if (std::abs(dq[i]) > 0.2) {
        std::cout << "Commanded joint position is too large from current "
                     "position, discarding plan..."
                  << std::endl;
        plan_local.reset();
        // Publish commanded position from previous control tick.
        for (int j = 0; j < kNumJoints; j++) {
          q_commanded[j] = iiwa_status.joint_position_commanded[j];
        }
      }
    }

    // construct and publish iiwa_command
    iiwa_command.utime = iiwa_status.utime;
    for (int i = 0; i < kNumJoints; i++) {
      iiwa_command.joint_position[i] = q_commanded(i);
    }
    publisher_lcm.publish(kLcmCommandChannel, &iiwa_command);
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
  auto T_ee = get_body_pose_in_world_frame(*tree_->FindBody("iiwa_link_ee"));
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
  {
    std::lock_guard<std::mutex> status_lock(robot_status_mutex_);
    iiwa_status_ = *status;
    has_received_new_status_ = true;
    is_waiting_for_first_robot_status_message_ = false;
    cur_time_us_ = iiwa_status_.utime;
    for (int i = 0; i < kNumJoints; i++) {
      current_robot_state_[i] = iiwa_status_.joint_position_measured[i];
      current_robot_state_[i + kNumJoints] =
          iiwa_status_.joint_velocity_estimated[i];
    }
  }
  cv_.notify_all();
}

void RobotPlanRunner::HandlePlan(const lcm::ReceiveBuffer *,
                                 const std::string &,
                                 const robotlocomotion::robot_plan_t *tape) {
  // Most of the code in this function is copied from
  // drake/examples/kuka_iiwa_arm/kuka_plan_runner.cc

  std::cout << "New plan received." << std::endl;
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
                                     Eigen::MatrixXd::Zero(kNumJoints, 1));
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

  const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
  auto plan_new_local = std::make_unique<JointSpaceTrajectoryPlan>(
      tree_, PPType::Cubic(input_time, knots, knot_dot, knot_dot));

  QueueNewPlan(std::move(plan_new_local));
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