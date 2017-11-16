#pragma once

#include <stdio.h>

#include "Eigen/Dense"

#include <opencv2/core/eigen.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

// From GraphCannySegmentation
// https://github.com/morpheus1820/graph-canny-segm/
static cv::Mat convertToColorMap(const cv::Mat& oneCHimg, double minVal = 0.0,
                                 double maxVal = 0.0,
                                 int colorMapType = cv::COLORMAP_HOT) {
  // if already 8 bit and we want to show gray image in ColorMap
  if (oneCHimg.depth() == CV_8U) {
    cv::Mat colorMatJ1;
    cv::applyColorMap(oneCHimg, colorMatJ1, colorMapType);

    return colorMatJ1;
  }
  // if other formats and we want to show gray image in ColorMap
  else {
    cv::Mat J1_gray_img;
    if (minVal == 0.0 && maxVal == 0.0) {
      cv::minMaxLoc(oneCHimg, &minVal, &maxVal);
    }
    oneCHimg.convertTo(J1_gray_img, CV_8U, 255.0 / (maxVal - minVal),
                       -minVal * 255.0 / (maxVal - minVal));
    cv::Mat colorMatJ1;
    cv::applyColorMap(J1_gray_img, colorMatJ1, colorMapType);
    // std::string name(winname+std::string(" J1"));
    return colorMatJ1;
  }
}

static cv::Mat matFromMatrixXd(const Eigen::MatrixXd matrix) {
  cv::Mat out = cv::Mat::zeros(matrix.rows(), matrix.cols(), CV_64FC1);
  eigen2cv(matrix, out);
  return out;
}

// from
// https://stackoverflow.com/questions/28562401/resize-an-image-to-a-square-but-keep-aspect-ratio-c-opencv
static cv::Mat resizeKeepAspectRatio(const cv::Mat& input,
                                     const cv::Size& dstSize,
                                     const cv::Scalar& bgcolor) {
  cv::Mat output;

  double h1 = dstSize.width * (input.rows / (double)input.cols);
  double w2 = dstSize.height * (input.cols / (double)input.rows);
  if (h1 <= dstSize.height) {
    cv::resize(input, output, cv::Size(dstSize.width, h1));
  } else {
    cv::resize(input, output, cv::Size(w2, dstSize.height));
  }

  int top = (dstSize.height - output.rows) / 2;
  int down = (dstSize.height - output.rows + 1) / 2;
  int left = (dstSize.width - output.cols) / 2;
  int right = (dstSize.width - output.cols + 1) / 2;

  cv::copyMakeBorder(output, output, top, down, left, right,
                     cv::BORDER_CONSTANT, bgcolor);

  return output;
}

static cv::Mat makeGridOfImages(const std::vector<cv::Mat>& images, int cols,
                                int min_gap_size, int width = 0,
                                int height = 0) {
  // let's first find out the maximum dimensions
  int max_width = 0;
  int max_height = 0;
  for (int i = 0; i < images.size(); i++) {
    // check if type is correct
    // you could actually remove that check and convert the image
    // in question to a specific type
    if (i > 0 && images[i].type() != images[i - 1].type()) {
      std::cerr << "WARNING:createOne failed, different types of images";
    }
    max_height = std::max(max_height, images[i].rows);
    max_width = std::max(max_width, images[i].cols);
  }
  // number of images in y direction
  int rows = std::ceil(images.size() / cols);

  // create our result-matrix
  cv::Mat result = cv::Mat::zeros(rows * max_height + (rows - 1) * min_gap_size,
                                  cols * max_width + (cols - 1) * min_gap_size,
                                  images[0].type());
  size_t i = 0;
  int current_height = 0;
  int current_width = 0;
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      // get the ROI in our result-image
      cv::Mat to(result,
                 cv::Range(current_height, current_height + images[i].rows),
                 cv::Range(current_width, current_width + images[i].cols));
      // copy the current image to the ROI
      images[i++].copyTo(to);
      current_width += max_width + min_gap_size;
    }
    // next line - reset width and update height
    current_width = 0;
    current_height += max_height + min_gap_size;
  }

  // resize it down
  if (width > 0 && height > 0)
    return resizeKeepAspectRatio(result, cv::Size(width, height), 0.0);
  else
    return result;
}