#include "rgbd_to_ros_pointcloud2_driver.h"

#include <memory>
#include <string>
#include <vector>

#include <zlib.h>

#include "drake/systems/sensors/image.h"

#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

using std::string;

namespace spartan {
namespace drake_simulation {
namespace iiwa_rlg_simulation {

using drake::systems::sensors::PixelType;
using drake::systems::sensors::Image;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::ImageDepth32F;

const int64_t kSecToMillisec = 1000000;

RgbdCameraRosDriver::RgbdCameraRosDriver(const string& color_frame_name,
                                         const string& depth_frame_name,
                                         const string& channel_prefix,
                                         ros::NodeHandle& nh)
    : color_frame_name_(color_frame_name),
      depth_frame_name_(depth_frame_name),
      image_transport(nh),
      rgb_image_publisher(
          image_transport.advertise(channel_prefix + "/rgb_raw", 1)),
      depth_image_publisher(
          image_transport.advertise(channel_prefix + "/depth_raw", 1)) {
  color_image_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::Value<ImageRgba8U>())
          .get_index();

  depth_image_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::Value<ImageDepth32F>())
          .get_index();
}

const InputPortDescriptor<double>& RgbdCameraRosDriver::color_image_input_port()
    const {
  return this->get_input_port(color_image_input_port_index_);
}

const InputPortDescriptor<double>& RgbdCameraRosDriver::depth_image_input_port()
    const {
  return this->get_input_port(depth_image_input_port_index_);
}

void RgbdCameraRosDriver::set_publish_period(double period) {
  LeafSystem<double>::DeclarePeriodicPublish(period);
}

void RgbdCameraRosDriver::DoPublish(
    const drake::systems::Context<double>& context,
    const std::vector<const drake::systems::PublishEvent<double>*>&) const {
  long long int utime =
      static_cast<int64_t>(context.get_time() * kSecToMillisec);

  const AbstractValue* color_image_value =
      this->EvalAbstractInput(context, color_image_input_port_index_);

  const AbstractValue* depth_image_value =
      this->EvalAbstractInput(context, depth_image_input_port_index_);

  if (color_image_value) {
    const ImageRgba8U& color_image = color_image_value->GetValue<ImageRgba8U>();

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
    color_image_msg.data.insert(
        color_image_msg.data.end(), &color_image.at(0, 0)[0],
        &(color_image.at(color_image_msg.width - 1,
                         color_image_msg.height - 1)[3]) +
            1);

    rgb_image_publisher.publish(color_image_msg);
  }

  if (depth_image_value) {
    const ImageDepth32F& depth_image =
        depth_image_value->GetValue<ImageDepth32F>();

    sensor_msgs::Image depth_image_msg;
    depth_image_msg.header.stamp = ros::Time::now();
    depth_image_msg.height = depth_image.height();
    depth_image_msg.width = depth_image.width();
    depth_image_msg.encoding = "rgba8";
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
                                          depth_image_msg.height - 1)[0]) +
                         1));
    depth_image_publisher.publish(depth_image_msg);
  }
}
}
}
}  // namespace spartan