#pragma once

#include <mutex>

#include "drake/lcm/drake_lcm.h"

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"

#include "system_utils.h"

class RgbdCameraDriver : drake::lcm::DrakeLcmMessageHandlerInterface {
 public:
  typedef std::pair<cv::Mat, cv::Mat> RgbdCameraData;

  RgbdCameraDriver(const std::string channel);

  RgbdCameraData get_latest_images() {
    image_lock_.lock();
    RgbdCameraData data({latest_rgb_image_.clone(), latest_depth_image_.clone()});
    image_lock_.unlock();
    return data;
  }
  double get_time_since_latest_images() {
    return getUnixTime() - last_receive_time_;
  }

 private:

  void HandleMessage(const std::string& channel,
                     const void* message_buffer, int message_size);

  drake::lcm::DrakeLcm lcm_;

  std::mutex image_lock_;
  cv::Mat latest_rgb_image_;
  cv::Mat latest_depth_image_;

  double last_receive_time_;
};