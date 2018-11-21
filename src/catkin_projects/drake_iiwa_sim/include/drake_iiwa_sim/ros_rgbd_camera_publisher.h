#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"

#include <tf/transform_broadcaster.h>
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

namespace drake_iiwa_sim {

class RosRgbdCameraPublisher : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RosRgbdCameraPublisher)

  /// @param color_frame_name The frame name used for color image.
  /// @param depth_frame_name The frame name used for depth image.
  RosRgbdCameraPublisher(
      const drake::systems::sensors::dev::RgbdCamera& rgbd_camera,
      const std::string& camera_name, double draw_period = 0.033);

  /**
   * Sets the publishing period of this system.
   */
  void set_publish_period(double period);

  /// Returns a descriptor of the input port containing a color image.
  const drake::systems::InputPort<double>& color_image_input_port() const {
    return color_image_input_port_;
  }

  /// Returns a descriptor of the input port containing a depth image.
  const drake::systems::InputPort<double>& depth_image_input_port() const {
    return depth_image_input_port_;
  }

  /// Returns a descriptor of the input port containing a label image.
  const drake::systems::InputPort<double>& label_image_input_port() const {
    return label_image_input_port_;
  }

  /// Returns a descriptor of the input port containing the camera base pose.
  const drake::systems::InputPort<double>& camera_base_pose_input_port() const {
    return camera_base_pose_input_port_;
  }

  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::PublishEvent<double>*>&) const;

 private:
  sensor_msgs::CameraInfo MakeCameraInfoMsg(
      const drake::systems::sensors::CameraInfo& camera_info);

  mutable sensor_msgs::CameraInfo rgb_info_msg_;
  std::string rgb_frame_name_;
  mutable sensor_msgs::CameraInfo depth_info_msg_;
  std::string depth_frame_name_;

  const drake::systems::InputPort<double>& color_image_input_port_;
  const drake::systems::InputPort<double>& depth_image_input_port_;
  const drake::systems::InputPort<double>& label_image_input_port_;
  const drake::systems::InputPort<double>& camera_base_pose_input_port_;

  const drake::systems::sensors::dev::RgbdCamera& rgbd_camera_;
  mutable ros::NodeHandle nh_;
  mutable tf::TransformBroadcaster br_;
  mutable image_transport::ImageTransport image_transport_;
  mutable image_transport::Publisher rgb_image_publisher_;
  mutable image_transport::Publisher depth_image_publisher_;
  mutable image_transport::Publisher label_image_publisher_;
  mutable ros::Publisher rgb_camera_info_publisher_;
  mutable ros::Publisher depth_camera_info_publisher_;
};

}  // Namespace drake_iiwa_sim