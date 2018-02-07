#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

#include "image_transport/image_transport.h"
#include "ros/ros.h"

namespace spartan {
namespace rlg_simulation {
namespace iiwa_rlg_simulation {

using namespace drake::systems;

class RgbdCameraRosDriver : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdCameraRosDriver)

  /// @param color_frame_name The frame name used for color image.
  /// @param depth_frame_name The frame name used for depth image.
  RgbdCameraRosDriver(const std::string& color_frame_name,
                      const std::string& depth_frame_name,
                      const std::string& channel_prefix, ros::NodeHandle& nh);

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePublishPeriodSec() for details about the semantics of
   * parameter `period`.
   * Mirrors `lcm_publisher_system`.
   */
  void set_publish_period(double period);

  /// Returns a descriptor of the input port containing a color image.
  const InputPortDescriptor<double>& color_image_input_port() const;

  /// Returns a descriptor of the input port containing a depth image.
  const InputPortDescriptor<double>& depth_image_input_port() const;

  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::PublishEvent<double>*>&) const;

 private:
  int color_image_input_port_index_{};
  int depth_image_input_port_index_{};

  std::string color_frame_name_;
  std::string depth_frame_name_;

  image_transport::ImageTransport image_transport;
  image_transport::Publisher rgb_image_publisher;
  image_transport::Publisher depth_image_publisher;
};

}
}
}  // namespace spartan