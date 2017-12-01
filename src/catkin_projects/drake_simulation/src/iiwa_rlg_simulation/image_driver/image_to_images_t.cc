#include "image_to_images_t.h"

#include <memory>
#include <string>
#include <vector>

#include <zlib.h>
#include "bot_core/image_t.hpp"
#include "bot_core/images_t.hpp"

#include "drake/systems/sensors/image.h"

using std::string;

namespace spartan {
namespace drake_simulation {
namespace iiwa_rlg_simulation {

using drake::systems::sensors::PixelType;
using drake::systems::sensors::Image;
using drake::systems::sensors::ImageRgba8U;
using drake::systems::sensors::ImageDepth32F;

const int64_t kSecToMillisec = 1000000;

template <PixelType kPixelType>
void Compress(const Image<kPixelType>& image, bot_core::image_t* msg) {
  msg->pixelformat = bot_core::image_t::PIXEL_FORMAT_ANY;

  const int source_size = image.width() * image.height() * image.kPixelSize;
  // The destination buf_size must be slightly larger than the source size.
  // http://refspecs.linuxbase.org/LSB_3.0.0/LSB-PDA/LSB-PDA/zlib-compress2-1.html
  size_t buf_size = source_size * 1.001 + 12;
  std::unique_ptr<uint8_t[]> buf(new uint8_t[buf_size]);

  auto compress_status = compress2(
      buf.get(), &buf_size, reinterpret_cast<const Bytef*>(image.at(0, 0)),
      source_size, Z_BEST_SPEED);

  DRAKE_DEMAND(compress_status == Z_OK);

  msg->data.resize(buf_size);
  msg->size = buf_size;
  memcpy(&msg->data[0], buf.get(), buf_size);
}

template <PixelType kPixelType>
void PackImageToImageT(const Image<kPixelType>& image, int64_t utime,
                       uint8_t pixel_format, const string& frame_name,
                       bot_core::image_t* msg) {
  // TODO(kunimatsu-tri) Fix seq here that is always set to zero.
  msg->utime = utime;
  msg->width = image.width();
  msg->height = image.height();
  msg->row_stride = image.kPixelSize * msg->width;
  msg->pixelformat = pixel_format;
  msg->nmetadata = 0;

  // TODO(kunimatsu-tri) Make compression optional and/or selectable.
  Compress(image, msg);
}

ImagesToLcmImagesT::ImagesToLcmImagesT(const string& color_frame_name,
                                       const string& depth_frame_name)
    : color_frame_name_(color_frame_name), depth_frame_name_(depth_frame_name) {
  color_image_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::Value<ImageRgba8U>())
          .get_index();

  depth_image_input_port_index_ =
      DeclareAbstractInputPort(drake::systems::Value<ImageDepth32F>())
          .get_index();

  images_t_msg_output_port_index_ =
      DeclareAbstractOutputPort(&ImagesToLcmImagesT::CalcImagesT).get_index();
}

const InputPortDescriptor<double>& ImagesToLcmImagesT::color_image_input_port()
    const {
  return this->get_input_port(color_image_input_port_index_);
}

const InputPortDescriptor<double>& ImagesToLcmImagesT::depth_image_input_port()
    const {
  return this->get_input_port(depth_image_input_port_index_);
}

const OutputPort<double>& ImagesToLcmImagesT::images_t_msg_output_port() const {
  return System<double>::get_output_port(images_t_msg_output_port_index_);
}

void ImagesToLcmImagesT::CalcImagesT(
    const drake::systems::Context<double>& context,
    bot_core::images_t* msg) const {
  msg->utime = static_cast<int64_t>(context.get_time() * kSecToMillisec);
  msg->n_images = 0;
  msg->image_types.clear();
  msg->images.clear();

  const AbstractValue* color_image_value =
      this->EvalAbstractInput(context, color_image_input_port_index_);

  const AbstractValue* depth_image_value =
      this->EvalAbstractInput(context, depth_image_input_port_index_);

  if (color_image_value) {
    const ImageRgba8U& color_image = color_image_value->GetValue<ImageRgba8U>();

    bot_core::image_t color_image_msg;
    PackImageToImageT(color_image, msg->utime,
                      bot_core::image_t::PIXEL_FORMAT_RGBA, color_frame_name_,
                      &color_image_msg);
    msg->images.push_back(color_image_msg);
    msg->image_types.push_back(
        4);  // 4 = msg->DEPTH_MM, causing linker trouble...
    msg->n_images++;
  }

  if (depth_image_value) {
    const ImageDepth32F& depth_image =
        depth_image_value->GetValue<ImageDepth32F>();
    bot_core::image_t depth_image_msg;
    PackImageToImageT(depth_image, msg->utime,
                      bot_core::image_t::PIXEL_FORMAT_FLOAT_GRAY32,
                      depth_frame_name_, &depth_image_msg);
    msg->images.push_back(depth_image_msg);
    msg->image_types.push_back(6);  // 6 = msg->LEFT, causing linker trouble...
    msg->n_images++;
  }
}

}
}
}