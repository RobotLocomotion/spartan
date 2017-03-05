//
// Created by manuelli on 1/30/17.
//

#ifndef DRAKE_SUPERBUILD_KUKA_IIWA_RESIDUAL_DETECTOR_TRANSLATOR_H
#define DRAKE_SUPERBUILD_KUKA_IIWA_RESIDUAL_DETECTOR_TRANSLATOR_H

#include <memory>
#include <iostream>
#include <Eigen/Dense>


#include "drake/lcmt_iiwa_status.hpp"
#include "LCMHandler.h"
#include <lcm/lcm-cpp.hpp>


namespace drake{
namespace examples{
namespace ContactParticleFilter{


  struct KukaIIWAStatusResidualConfig{
    std::string publish_channel;
    std::string receive_channel;
  };

  struct KukaIIWAStatusResidualState{
    drake::lcmt_iiwa_status last_msg;
    Eigen::VectorXd residual;
  };

  KukaIIWAStatusResidualConfig parseConfig(std::string config_filename);

  class KukaIIWAStatusToResidual {
  public:
    LCMHandler lcm_handler_;
    KukaIIWAStatusToResidual(KukaIIWAStatusResidualConfig config);
//    ~KukaIIWAStatusToResidual(){};

    void Start();

  private:

    KukaIIWAStatusResidualConfig config_;
    KukaIIWAStatusResidualState state_;
    std::vector<std::string> joint_names_;
    int num_joints_;
    bool initialized_ = false;
    lcm::LCM lcm_;
    void onIIWAStatus(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                      const drake::lcmt_iiwa_status *msg);

    void SetupSubscribers();
    void PublishResidual();
    void Update();
    void Initialize();

  };

}// ContactParticleFilter
}// examples
}// drake


#endif //DRAKE_SUPERBUILD_KUKA_IIWA_RESIDUAL_DETECTOR_TRANSLATOR_H
