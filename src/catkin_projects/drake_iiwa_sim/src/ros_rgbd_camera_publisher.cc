#include "drake_iiwa_sim/ros_rgbd_camera_publisher.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake_iiwa_sim {

using drake::systems::Context;
using drake::systems::sensors::dev::RgbdCamera;
using drake::systems::Value;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageLabel16I;

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
  // Default from carmine, what do I do instead of this? TODO(gizatt)
  info_msg.D = std::vector<double>({-0.05031626410741169, -0.2754641106388708,
                                    0.0003722647488938247,
                                    0.0003210956898043667, 0.2978803215933795});
  const auto K = camera_info.intrinsic_matrix();
  std::copy_n(K.data(), 9, info_msg.K.begin());
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  std::copy_n(R.data(), 9, info_msg.R.begin());
  Eigen::MatrixXd P(3, 4);
  P.block<3, 3>(0, 0) = K;
  P.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 1.);
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
  depth_image_msg.encoding = "32FC1";
  depth_image_msg.is_bigendian = false;
  depth_image_msg.step = depth_image_msg.width * 4;
  depth_image_msg.data.clear();
  depth_image_msg.data.insert(
      depth_image_msg.data.end(), (unsigned char*)&depth_image.at(0, 0)[0],
      (unsigned char*)(&(depth_image.at(depth_image_msg.width - 1,
                                        depth_image_msg.height - 1)[0]) +
                       1));

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