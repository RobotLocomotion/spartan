#include <robot_plan_runner/plan_runner.h>

namespace drake {
namespace robot_plan_runner {

RobotPlanRunner::RobotPlanRunner()
    : plan_number_(0), plan_(nullptr), new_plan_(nullptr) {
  tree_ = std::make_unique<RigidBodyTreed>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_no_collision.urdf"),
      multibody::joints::kFixed, tree_.get());
  DRAKE_DEMAND(kNumJoints == tree_->get_num_positions());
  DRAKE_DEMAND(kNumJoints == tree_->get_num_actuators());
  receiver_lcm_.subscribe(kLcmStatusChannel, &RobotPlanRunner::HandleStatus,
                          this);
  current_robot_state_.resize(kNumJoints * 2, 1);
  has_received_new_status_ = false;
  waiting_for_first_message_ = true;
}

void RobotPlanRunner::Start() {
  publish_thread_ = std::thread(&RobotPlanRunner::PublishCommand, this);
  subscriber_thread_ =
      std::thread(&RobotPlanRunner::ReceiveRobotStatus, this);
}

void RobotPlanRunner::Stop() {
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  if (subscriber_thread_.joinable()) {
    subscriber_thread_.join();
  }
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

  while (true) {
    // Call lcm handle until at least one status message is
    // processed.
    while (0 == receiver_lcm_.handleTimeout(10) ||
        waiting_for_first_message_) {
      // Print something here so users know no LCM message has been received.
    }
  }
}

void RobotPlanRunner::PublishCommand() {
  robot_status_mutex_.lock();
  std::cout << "Command publisher thread starting on thread "
            << std::this_thread::get_id() << std::endl;
  robot_status_mutex_.unlock();

  int64_t start_time_us = -1;
  int64_t cur_time_us = -1;
  double cur_plan_time_s = -1;

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
    // the
    // subscriber thread.
    std::unique_lock<std::mutex> status_lock(robot_status_mutex_);
    cv_.wait(status_lock,
             std::bind(&RobotPlanRunner::has_received_new_status, this));
    has_received_new_status_ = false;
    cur_time_us = cur_time_us_;
    current_robot_state = current_robot_state_;

    // Calling unlock is necessary because when cv_.wait() returns, this
    // thread acquires the mutex, preventing the receiver thread from
    // executing.
    status_lock.unlock();

    if (plan_number_ == 0) {
      // This block should only run once, right after the infinite while loop
      // starts.
      std::cout << "Generating first plan(holding current position)..."
                << std::endl;
      new_plan_ = JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(
          tree_, current_robot_state_.head(kNumJoints));
    }

    if (new_plan_) {
      robot_plan_mutex_.lock();
      plan_ = std::move(new_plan_);
      robot_plan_mutex_.unlock();

      plan_number_++;
      start_time_us = cur_time_us;
      is_cur_plan_terminated_ = false;
      std::cout << "Starting plan No. " << plan_number_ << std::endl;
    }

    cur_plan_time_s = static_cast<double>(cur_time_us - start_time_us) / 1e6;
    plan_->Step(current_robot_state, cur_plan_time_s, &q_commanded,
                &v_commanded);

    // The following check is good to have but I haven't figured out the right
    // place for it.
    // Stop if commanded q is "too different" from current q.
    //      Eigen::VectorXd dq = q_commanded -
    //      current_robot_state_.head(kNumJoints);
    //      for (int i = 0; i < kNumJoints; i++) {
    //        if (std::abs(dq[i]) > 0.1) {
    //          is_cur_plan_terminated_ = true;
    //          break;
    //        }
    //      }
    //
    //      if (is_cur_plan_terminated_) {
    //        std::cout << "Difference between q_commanded and q too large."
    //                     " Aborting plan "
    //                  << cur_plan_number << " and starting a new blank
    //                  plan."
    //                  << std::endl;
    //        std::lock_guard<std::mutex> lock(robot_plan_mutex_);
    //        new_plan_ =
    //        JointSpaceTrajectoryPlan::MakeHoldCurrentPositionPlan(
    //            tree_, current_robot_state_.head(kNumJoints));
    //        continue;
    //      }

    // construct and publish iiwa_command
    iiwa_command.utime = cur_time_us;
    for (int i = 0; i < kNumJoints; i++) {
      iiwa_command.joint_position[i] = q_commanded(i);
    }
    publihser_lcm_.publish(kLcmCommandChannel, &iiwa_command);
  }
}

void RobotPlanRunner::MoveToJointPosition(const Eigen::Ref<const Eigen::VectorXd> q_final,
                         double duration) {
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

void RobotPlanRunner::HandleStatus(const lcm::ReceiveBuffer *, const std::string &,
                  const lcmt_iiwa_status *status) {
  {
    std::lock_guard<std::mutex> status_lock(robot_status_mutex_);
    iiwa_status_ = *status;
    has_received_new_status_ = true;
    waiting_for_first_message_ = false;
    cur_time_us_ = iiwa_status_.utime;
    for (int i = 0; i < kNumJoints; i++) {
      current_robot_state_[i] = iiwa_status_.joint_position_measured[i];
      current_robot_state_[i + kNumJoints] =
          iiwa_status_.joint_velocity_estimated[i];
    }
  }
  cv_.notify_all();
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