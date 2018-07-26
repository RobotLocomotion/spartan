//
// Created by manuelli on 7/26/18.
//

#include "drake_robot_control/force_guard.h"

namespace spartan{
namespace drake_robot_control{

TotalExternalTorqueGuard::TotalExternalTorqueGuard(const double& external_torque_norm_threshold):
    ForceGuard(ForceGuardType::TOTAL_EXTERNAL_TORQUE), threshold_(external_torque_norm_threshold){};

std::pair<bool, double> TotalExternalTorqueGuard::EvaluateGuard(const KinematicsCache<double>& cache_, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &tau_external){

  double fraction = tau_external.norm()/threshold_;
  bool guard_triggered = fraction > 1.0;

  // set the global state
  if (guard_triggered){
    has_been_triggered_ = true;
  }

  return std::make_pair(guard_triggered, fraction);
}

ExternalForceGuard::ExternalForceGuard(const RigidBodyTreed& tree, int idx_body, int idx_world, int idx_expressed_in, const Eigen::Ref<const Eigen::Vector3d> &force):
    ForceGuard(ForceGuardType::EXTERNAL_FORCE), tree_(tree), force_(force), idx_body_(idx_body),
    idx_world_(idx_world), idx_expressed_in_(idx_expressed_in){

  twist_external_ = Eigen::Matrix<double, 6,1>::Zero();

}

std::pair<bool, double> ExternalForceGuard::EvaluateGuard(const KinematicsCache<double>& cache_, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &tau_external){

  // compute expressed_in to body  transform
  H_body_expressed_in_ = tree_.relativeTransform(cache_, idx_body_, idx_expressed_in_);

  // rotate force to be in body frame
  twist_external_.tail(3) = H_body_expressed_in_.linear() * force_;

  Eigen::MatrixXd J = tree_.geometricJacobian(cache_, idx_world_, idx_body_, idx_body_);
  Eigen::VectorXd torque_external_threshold = J.transpose() *  twist_external_;

  // hack to avoid division by zero
  double fraction = tau_external.norm()/(torque_external_threshold.norm() + 1e-5);
  bool guard_triggered = fraction > 1.0;

  if (guard_triggered){
    has_been_triggered_ = true;

//    std::cout << "\n\n-------ExternalForceGuard Triggered-------\n\n" << std::endl;
//    std::cout << "torque_external_threshold:\n" << torque_external_threshold << std::endl;
//    std::cout << "\ntau_external:\n" << tau_external << std::endl;
//    std::cout << "\ntwist in body frame:\n" << twist_external_ << std::endl;
  }

  return std::make_pair(guard_triggered, fraction);
}

std::pair<bool, std::pair<double, std::shared_ptr<ForceGuard>>> ForceGuardContainer::EvaluateGuards(const KinematicsCache<double>& cache_, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &tau_external){

  double largest_fraction;
  bool guard_triggered = false;
  std::shared_ptr<ForceGuard> guard;
  std::pair<bool, double> result;

  for (int i = 0; i < guards_.size(); i++){
    result = guards_[i]->EvaluateGuard(cache_, q, tau_external);

    if (i == 0){
      guard_triggered = result.first;
      largest_fraction = result.second;
      guard = guards_[i];
    } else if (result.second > largest_fraction){
      guard_triggered = result.first;
      largest_fraction = result.second;
      guard = guards_[i];
    }
  }

  if (guard_triggered){
    has_been_triggered_ = true;
    triggered_guard_ = guard;
  }

  return std::make_pair(guard_triggered, std::make_pair(guard_triggered, guard));
}



}//spartan
}//drake_robot_control