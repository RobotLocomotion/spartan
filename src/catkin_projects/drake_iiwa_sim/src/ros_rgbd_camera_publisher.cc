#include "drake_iiwa_sim/ros_rgbd_camera_publisher.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake_iiwa_sim {

using drake::systems::Context;
using drake::systems::sensors::dev::RgbdCamera;
using drake::systems::Value;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageLabel16I;

RosRgbdCameraPublisher::RosRgbdCameraPublisher(
    const RgbdCamera& rgbd_camera, const std::string& channel_prefix,
    double draw_period)
    : rgbd_camera_(rgbd_camera),
      image_transport_(nh_),
      rgb_image_publisher_(
          image_transport_.advertise(channel_prefix + "/rgb/image_raw", 1)),
      depth_image_publisher_(
          image_transport_.advertise(channel_prefix + "/depth/image_raw", 1)),
      label_image_publisher_(
          image_transport_.advertise(channel_prefix + "/label/image", 1)),
      color_image_input_port_(DeclareAbstractInputPort(
          drake::systems::kUseDefaultName, Value<ImageRgba8U>())),
      depth_image_input_port_(DeclareAbstractInputPort(
          drake::systems::kUseDefaultName, Value<ImageDepth32F>())),
      label_image_input_port_(DeclareAbstractInputPort(
          drake::systems::kUseDefaultName, Value<ImageLabel16I>())),
      camera_base_pose_input_port_(DeclareVectorInputPort(
          drake::systems::kUseDefaultName,
          drake::systems::rendering::PoseVector<double>())) {
  DeclarePeriodicPublish(draw_period, 0.0);
  drake::systems::PublishEvent<double> init_event(
      drake::systems::Event<double>::TriggerType::kInitialization);
}

void RosRgbdCameraPublisher::DoPublish(
    const Context<double>& context,
    const std::vector<const drake::systems::PublishEvent<double>*>& event)
    const {
  printf("DoPublish in RosRgbdCameraPublisher\n");
  ros::spinOnce();
}

}  // namespace drake_iiwa_sim