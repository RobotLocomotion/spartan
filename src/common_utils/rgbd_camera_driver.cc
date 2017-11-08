#include "rgbd_camera_driver.h"
#include <zlib.h>
#include "cv-utils/image_utils/jpeg.h"
#include "lcmtypes/bot_core/image_t.hpp"
#include "lcmtypes/bot_core/images_t.hpp"

using namespace std;

#include <limits>

RgbdCameraDriver::RgbdCameraDriver(const std::string channel)
    : last_receive_time_(-std::numeric_limits<double>::infinity()) {
  lcm_.Subscribe(channel, this);
  lcm_.StartReceiveThread();
}

void RgbdCameraDriver::HandleMessage(const std::string& channel,
                                     const void* message_buffer,
                                     int message_size) {
  bot_core::images_t image_array;
  image_array.decode(message_buffer, 0, message_size);

  for (int image_i = 0; image_i < image_array.images.size(); image_i++) {
    const auto& image = image_array.images[image_i];

    int w = image.width;
    int h = image.height;

    if (w == 0 || h == 0) {
      printf(
          "Got an images_t message with zero width or height, skipping...\n");
      continue;
    }

    int pixelFormat = image.pixelformat;

    bool isZlibCompressed = false;
    if (pixelFormat == bot_core::image_t::PIXEL_FORMAT_INVALID) {
      pixelFormat = bot_core::image_t::PIXEL_FORMAT_LE_GRAY16;
      isZlibCompressed = true;
    }

    if (pixelFormat == bot_core::image_t::PIXEL_FORMAT_RGB ||
        pixelFormat == bot_core::image_t::PIXEL_FORMAT_MJPEG) {
      cv::Mat rawData(1, image.size, CV_8UC1, (void*)image.data.data());
      image_lock_.lock();
      latest_rgb_image_ = cv::imdecode(rawData, CV_LOAD_IMAGE_COLOR);
      image_lock_.unlock();
    } else if (pixelFormat == bot_core::image_t::PIXEL_FORMAT_LE_GRAY16) {
      vector<uint8_t> output_buffer;
      if (isZlibCompressed) {
        output_buffer.resize(w * h * 2);
        unsigned long len = w * h * 2;
        uncompress(output_buffer.data(), &len, image.data.data(), image.size);
      } else {
        output_buffer.assign(image.data.data(), image.data.data() + image.size);
      }

      cv::Mat rawData(h, w, CV_16UC1, (void*)output_buffer.data());

      image_lock_.lock();
      latest_depth_image_ = rawData.clone();
      image_lock_.unlock();
    } else {
      printf("unhandled image pixelformat %d\n", pixelFormat);
    }
  }

  last_receive_time_ = getUnixTime();
}