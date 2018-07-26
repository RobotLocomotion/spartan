#pragma once

#include <string>
#include <utility>
#include <Eigen/Dense>

// drake
#include <drake/multibody/rigid_body_tree.h>

namespace spartan{
namespace drake_robot_control{

enum ForceGuardType{
TOTAL_EXTERNAL_TORQUE,
EXTERNAL_FORCE,
};

/// Generic ForceGuard class.
/// Contains a single method, EvaluateGuard, which checks to see if the guard has been triggered,
/// and additionally reports the fraction of how much it has been triggered
class ForceGuard{
 public:
  ForceGuard(const ForceGuardType & type):type_(type), has_been_triggered_(false){};

  /**
   * Checks whether the guard has been triggered
   * Reports the fraction of the way it is to being triggered
   * @param cache_
   * @param q
   * @param tau_external
   * @return
   */
  virtual std::pair<bool, double> EvaluateGuard(const KinematicsCache<double>& cache_, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &tau_external) = 0;

  inline bool HasBeenTriggered(){
    return has_been_triggered_;
  }

  inline ForceGuardType get_type(){
    return type_;
  }

 protected:
  ForceGuardType type_;
  bool has_been_triggered_;
};

/// Guard on the norm of joint_torque_external
class TotalExternalTorqueGuard : public ForceGuard{
 public:
  TotalExternalTorqueGuard(const double& external_torque_norm_threshold);

  std::pair<bool, double> EvaluateGuard(const KinematicsCache<double>& cache_, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &tau_external) override ;

 private:
  const double threshold_;// total external torque threshold
};

/// Guard on the force applied at a specific location on a body.
/// Checks whether joint_torque_external.norm() > threshold_torque_external.norm()
/// where threshold_torque_external is the external torque gotten by applying the specific
/// force to the specified body location
class ExternalForceGuard : public ForceGuard{
 public:
  ExternalForceGuard(const RigidBodyTreed& tree, double force_threshold, int idx_body, int idx_world, int idx_expressed_in, const Eigen::Ref<const Eigen::Vector3d> &force_direction);

  std::pair<bool, double> EvaluateGuard(const KinematicsCache<double>& cache_, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &tau_external) override ;

 private:
  const RigidBodyTreed& tree_;

  const double force_threshold_;
  const int idx_world_;
  const int idx_body_;
  const int idx_expressed_in_;
  Eigen::Vector3d force_direction_;
  Eigen::Matrix<double, 6,1> twist_external_;
  Eigen::Isometry3d H_body_expressed_in_;

};

//struct ForceGuardContainerResult{
//  std::shared_ptr<ForceGuard> guard;
//  double fraction
//};

class ForceGuardContainer{
 public:
  ForceGuardContainer();

  inline
  void AddGuard(std::shared_ptr<ForceGuard> guard){
    guards_.push_back(guard);
  }

  std::pair<bool, std::pair<double, std::shared_ptr<ForceGuard>>> EvaluateGuards(const KinematicsCache<double>& cache_, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &tau_external);

 private:
  std::vector<std::shared_ptr<ForceGuard>> guards_;
};

}//spartan
}//drake_robot_control