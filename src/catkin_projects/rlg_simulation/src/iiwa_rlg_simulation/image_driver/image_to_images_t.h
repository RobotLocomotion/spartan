#pragma once

#include <string>

#include "bot_core/images_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace spartan {
namespace rlg_simulation {
namespace iiwa_rlg_simulation {

using namespace drake::systems;

/// An ImagesToLcmImagesT takes as input a ImageBgra8U, ImageDepth32F.
/// This system outputs an AbstractValue containing a
/// `Value<bot_core::images_t>` LCM message that defines an array
/// of images (image_t). This message can then be sent to other processes that
/// sbscribe it using LcmPublisherSystem.

/// NOTE: This is a stopgap to fix a compatibility issue with Drake:
/// drake prefers robotlocomotion::image_array_t, while openni2-lcm-driver
/// prefers bot_core:images_t. We ought to move to the robotlocomotion type
/// eventually, as it is superior.
class ImagesToLcmImagesT : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImagesToLcmImagesT)

  /// A %ImagesToLcmImagesT constructor.
  ///
  /// @param color_frame_name The frame name used for color image.
  /// @param depth_frame_name The frame name used for depth image.
  ImagesToLcmImagesT(const std::string& color_frame_name,
                     const std::string& depth_frame_name);

  /// Returns a descriptor of the input port containing a color image.
  const InputPortDescriptor<double>& color_image_input_port() const;

  /// Returns a descriptor of the input port containing a depth image.
  const InputPortDescriptor<double>& depth_image_input_port() const;

  /// Returns a descriptor of the abstract valued output port that contains a
  /// `Value<bot_core::images_t>`.
  const OutputPort<double>& images_t_msg_output_port() const;

 private:
  void CalcImagesT(const drake::systems::Context<double>& context,
                   bot_core::images_t* msg) const;

  int color_image_input_port_index_{};
  int depth_image_input_port_index_{};
  int images_t_msg_output_port_index_{};

  std::string color_frame_name_;
  std::string depth_frame_name_;
};
}
}
}  // namespace spartan