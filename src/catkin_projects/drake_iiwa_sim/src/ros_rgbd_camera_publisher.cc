#include "drake_iiwa_sim/ros_rgbd_camera_publisher.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake_iiwa_sim {

using drake::systems::Context;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::dev::RgbdCamera;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageLabel16I;
using drake::systems::rendering::PoseVector;
using drake::systems::Value;

RosRgbdCameraPublisher::RosRgbdCameraPublisher(const RgbdCamera& rgbd_camera,
                                               const std::string& camera_name,
                                               double draw_period)
    : rgbd_camera_(rgbd_camera),
      image_transport_(nh_),
      rgb_image_publisher_(image_transport_.advertise(
          "/camera_" + camera_name + "/rgb/image_raw", 1)),
      depth_image_publisher_(image_transport_.advertise(
          "/camera_" + camera_name + "/depth/image_raw", 1)),
      label_image_publisher_(image_transport_.advertise(
          "/camera_" + camera_name + "/label/image", 1)),
      rgb_camera_info_publisher_(nh_.advertise<sensor_msgs::CameraInfo>(
          "/camera_" + camera_name + "/rgb/camera_info", 1)),
      depth_camera_info_publisher_(nh_.advertise<sensor_msgs::CameraInfo>(
          "/camera_" + camera_name + "/depth/camera_info", 1)),
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

  // Prepare camera_info messages from th RGBD camera
  rgb_info_msg_ = MakeCameraInfoMsg(rgbd_camera.color_camera_info());
  rgb_frame_name_ = "camera_" + camera_name + "_rgb_optical_frame";
  depth_frame_name_ = "camera_" + camera_name + "_depth_optical_frame";
  depth_info_msg_ = MakeCameraInfoMsg(rgbd_camera.depth_camera_info());
}

sensor_msgs::CameraInfo RosRgbdCameraPublisher::MakeCameraInfoMsg(
    const CameraInfo& camera_info) {
  sensor_msgs::CameraInfo info_msg;

  info_msg.width = camera_info.width();
  info_msg.height = camera_info.height();
  info_msg.distortion_model = "plumb_bob";
  // Default no distortion TODO(gizatt)
  info_msg.D = std::vector<double>({0., 0., 0., 0., 0.});
  // Row-major ordering to make serialization come out in the
  // correct order.
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K =
      camera_info.intrinsic_matrix();
  std::copy_n(K.data(), 9, info_msg.K.begin());
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R = Eigen::Matrix3d::Identity();
  std::copy_n(R.data(), 9, info_msg.R.begin());
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> P =
      Eigen::Matrix<double, 3, 4, Eigen::RowMajor>::Zero();
  P.block<3, 3>(0, 0) = camera_info.intrinsic_matrix();
  std::copy_n(P.data(), 12, info_msg.P.begin());
  return info_msg;
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

  std_msgs::Header now_header;
  now_header.stamp = ros::Time::now();

  const PoseVector<double>* const pose_vector =
      dynamic_cast<const PoseVector<double>*>(this->EvalVectorInput(
          context, camera_base_pose_input_port_.get_index()));
  DRAKE_DEMAND(pose_vector);
  auto translation = pose_vector->get_translation();
  auto rotation = pose_vector->get_rotation();
  tf::Transform transform;
  transform.setOrigin(
      tf::Vector3(translation.x(), translation.y(), translation.z()));
  tf::Quaternion q(rotation.x(), rotation.y(), rotation.z(), rotation.w());
  transform.setRotation(q);
  br_.sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), "base", "drake_iiwa_camera_origin"));

  const auto& color_image = color_image_abstract->GetValue<ImageRgba8U>();
  sensor_msgs::Image color_image_msg;
  color_image_msg.height = color_image.height();
  color_image_msg.width = color_image.width();
  color_image_msg.encoding = "rgba8";
  color_image_msg.is_bigendian = false;
  color_image_msg.step = 4 * color_image_msg.width;
  color_image_msg.data.clear();
  color_image_msg.data.insert(color_image_msg.data.end(),
                              &color_image.at(0, 0)[0],
                              &(color_image.at(color_image_msg.width - 1,
                                               color_image_msg.height - 1)[3]) +
                                  1);

  now_header.frame_id = rgb_frame_name_;
  color_image_msg.header = now_header;
  rgb_image_publisher_.publish(color_image_msg);
  rgb_info_msg_.header = now_header;
  rgb_camera_info_publisher_.publish(rgb_info_msg_);

  const auto& depth_image = depth_image_abstract->GetValue<ImageDepth32F>();
  sensor_msgs::Image depth_image_msg;
  depth_image_msg.height = depth_image.height();
  depth_image_msg.width = depth_image.width();
  depth_image_msg.encoding = "16UC1";
  depth_image_msg.is_bigendian = false;
  depth_image_msg.step = depth_image_msg.width * 2;
  depth_image_msg.data.clear();
  // Convert to 16-bit unsigned int, with millimeter units.
  // (This appears to be the standard, at least for the OpenNI driver.)
  depth_image_msg.data.resize(depth_image_msg.width * depth_image_msg.height *
                              2);
  int ind = 0;
  for (int i = 0; i < depth_image_msg.height; i++) {
    for (int j = 0; j < depth_image_msg.width; j++) {
      uint16_t depth_pixel = (uint16_t)(depth_image.at(j, i)[0] * 1000);
      // Little-endian, as set above.
      depth_image_msg.data[ind] = depth_pixel & 0xff;
      depth_image_msg.data[ind + 1] = (depth_pixel >> 8) & 0xff;
      ind += 2;
    }
  }

  depth_image_msg.header = now_header;
  now_header.frame_id = depth_frame_name_;
  depth_image_publisher_.publish(depth_image_msg);
  depth_info_msg_.header = now_header;
  depth_camera_info_publisher_.publish(depth_info_msg_);

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