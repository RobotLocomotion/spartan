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
  const auto& color_image_abstract =
      this->EvalAbstractInput(context, color_image_input_port_.get_index());
  const auto& depth_image_abstract =
      this->EvalAbstractInput(context, depth_image_input_port_.get_index());

  /*
  const auto& label_image_abstract =
      this->EvalAbstractInput(context, label_image_input_port_.get_index());
  */
  if (!(color_image_abstract && depth_image_abstract)) {
    printf("Full frame not rendered yet? Skipping.");
    return;
  }

  const auto& color_image = color_image_abstract->GetValue<ImageRgba8U>();
  sensor_msgs::Image color_image_msg;
  color_image_msg.header.stamp = ros::Time::now();
  color_image_msg.height = color_image.height();
  color_image_msg.width = color_image.width();
  color_image_msg.encoding = "rgba8";
  color_image_msg.is_bigendian = false;
  color_image_msg.step = 4 * color_image_msg.width;
  color_image_msg.data.clear();

  // Copy image data into the message
  // by using the std::vector insert method,
  // with pointers to the first entry in the color image data,
  // and the last entry in the color image data.
  color_image_msg.data.insert(color_image_msg.data.end(),
                              &color_image.at(0, 0)[0],
                              &(color_image.at(color_image_msg.width - 1,
                                               color_image_msg.height - 1)[3]) +
                                  1);

  rgb_image_publisher_.publish(color_image_msg);

  const auto& depth_image = depth_image_abstract->GetValue<ImageDepth32F>();
  sensor_msgs::Image depth_image_msg;
  depth_image_msg.header.stamp = ros::Time::now();
  depth_image_msg.height = depth_image.height();
  depth_image_msg.width = depth_image.width();
  depth_image_msg.encoding = "32FC1";
  depth_image_msg.is_bigendian = false;
  depth_image_msg.step = depth_image_msg.width * 4;
  depth_image_msg.data.clear();

  // Copy image data into the message
  // by using the std::vector insert method,
  // with pointers to the first entry in the depth image data,
  // and the last entry in the depth image data.
  // The pointer arithmetic is tragic...
  depth_image_msg.data.insert(
      depth_image_msg.data.end(), (unsigned char*)&depth_image.at(0, 0)[0],
      (unsigned char*)(&(depth_image.at(depth_image_msg.width - 1,
                                        depth_image_msg.height - 1)[3]) +
                       1));
  depth_image_publisher_.publish(depth_image_msg);

  /*
  const auto& label_image = label_image_abstract->GetValue<ImageLabel16I>();
  sensor_msgs::Image label_image_msg;
  label_image_msg.header.stamp = ros::Time::now();
  label_image_msg.height = label_image.height();
  label_image_msg.width = label_image.width();
  label_image_msg.encoding = "16SC1";
  label_image_msg.is_bigendian = false;
  label_image_msg.step = label_image_msg.width * 2;
  label_image_msg.data.clear();

  // Copy image data into the message
  // by using the std::vector insert method,
  // with pointers to the first entry in the depth image data,
  // and the last entry in the depth image data.
  // The pointer arithmetic is tragic...
  label_image_msg.data.insert(
      label_image_msg.data.end(), (unsigned char*)&depth_image.at(0, 0)[0],
      (unsigned char*)(&(depth_image.at(label_image_msg.width - 1,
                                        label_image_msg.height - 1)[2]) +
                       1));
  label_image_publisher_.publish(label_image_msg);
  */

  ros::spinOnce();
}

}  // namespace drake_iiwa_sim