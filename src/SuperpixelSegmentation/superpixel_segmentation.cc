// Copyright 2014-2015 Isis Innovation Limited and the authors of gSLICr

#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "NVTimer.h"
#include "gSLICr_Lib/gSLICr.h"

#include "Eigen/Dense"
#include "Eigen/Eigenvalues"
#include "Eigen/Sparse"

#include <opencv2/core/eigen.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include "KMeansRexCoreInterface.h"
#include "common_utils/rgbd_camera_driver.h"

using namespace std;
using namespace cv;
using namespace Eigen;

cv::Mat convert_depth_image_to_tricky_rgb(const Mat& inimg) {
  cv::Mat out = cv::Mat::zeros(inimg.rows, inimg.cols, CV_8UC3);  // RGB img out

  for (int y = 0; y < inimg.rows; y++) {
    for (int x = 0; x < inimg.cols; x++) {
      // Valid if it's nonzero (sensor returns zero for unknown areas)
      short unsigned int valid =
          (inimg.at<short unsigned int>(y, x) > 100) * 0xff;

      // One channel (b) stores the low byte, or zero if the pixel is invalid
      out.at<Vec3b>(y, x)[0] =
          (unsigned char)(inimg.at<short unsigned int>(y, x) & valid);
      // The other channel (g) stores the high byte, or zero if this pixel is
      // invalid
      out.at<Vec3b>(y, x)[1] =
          (unsigned char)((inimg.at<short unsigned int>(y, x) >> 8) & valid);
      // The last channel (r) stores the validity
      out.at<Vec3b>(y, x)[2] = 0xff - valid;
    }
  }

  return out;
}

void load_image(const Mat& inimg, gSLICr::UChar4Image* outimg) {
  gSLICr::Vector4u* outimg_ptr = outimg->GetData(MEMORYDEVICE_CPU);

  for (int y = 0; y < outimg->noDims.y; y++)
    for (int x = 0; x < outimg->noDims.x; x++) {
      int idx = x + y * outimg->noDims.x;
      outimg_ptr[idx].b = inimg.at<Vec3b>(y, x)[0];
      outimg_ptr[idx].g = inimg.at<Vec3b>(y, x)[1];
      outimg_ptr[idx].r = inimg.at<Vec3b>(y, x)[2];
    }
}

void load_image(const gSLICr::UChar4Image* inimg, Mat& outimg) {
  const gSLICr::Vector4u* inimg_ptr = inimg->GetData(MEMORYDEVICE_CPU);

  for (int y = 0; y < inimg->noDims.y; y++)
    for (int x = 0; x < inimg->noDims.x; x++) {
      int idx = x + y * inimg->noDims.x;
      outimg.at<Vec3b>(y, x)[0] = inimg_ptr[idx].b;
      outimg.at<Vec3b>(y, x)[1] = inimg_ptr[idx].g;
      outimg.at<Vec3b>(y, x)[2] = inimg_ptr[idx].r;
    }
}

void load_image(const gSLICr::IntImage* inimg, Mat& outimg) {
  const int* inimg_ptr = inimg->GetData(MEMORYDEVICE_CPU);

  for (int y = 0; y < inimg->noDims.y; y++)
    for (int x = 0; x < inimg->noDims.x; x++) {
      int idx = x + y * inimg->noDims.x;
      outimg.at<int>(y, x) = inimg_ptr[idx];
    }
}

typedef Map<const Matrix<int, -1, -1, RowMajor>> LabelMatrix;
LabelMatrix convert_label_map_image(const gSLICr::IntImage* idx_img) {
  int width = idx_img->noDims.x;
  int height = idx_img->noDims.y;
  printf("Created with w %d, h %d\n", width, height);
  const int* data_ptr = idx_img->GetData(MEMORYDEVICE_CPU);
  return LabelMatrix(data_ptr, height, width);
}

// From GraphCannySegmentation
// https://github.com/morpheus1820/graph-canny-segm/
cv::Mat convertToColorMap(const cv::Mat& oneCHimg, double minVal, double maxVal,
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
    oneCHimg.convertTo(J1_gray_img, CV_8U, 255.0 / (maxVal - minVal),
                       -minVal * 255.0 / (maxVal - minVal));
    cv::Mat colorMatJ1;
    cv::applyColorMap(J1_gray_img, colorMatJ1, colorMapType);
    // std::string name(winname+std::string(" J1"));
    return colorMatJ1;
  }
}

cv::Mat matFromMatrixXd(const MatrixXd matrix) {
  cv::Mat out = cv::Mat::zeros(matrix.rows(), matrix.cols(), CV_64FC1);
  eigen2cv(matrix, out);
  return out;
}

// from
// https://stackoverflow.com/questions/28562401/resize-an-image-to-a-square-but-keep-aspect-ratio-c-opencv
cv::Mat resizeKeepAspectRatio(const cv::Mat& input, const cv::Size& dstSize,
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

void showGridOfImages(const std::vector<cv::Mat>& images, int cols,
                      int min_gap_size, const string& name, int width,
                      int height) {
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
  imshow(name, resizeKeepAspectRatio(result, Size(width, height), 0.0));
}

int main() {
  RgbdCameraDriver camera_driver("OPENNI_FRAME");

  // gSLICr settings
  gSLICr::objects::settings my_settings_rgb;
  my_settings_rgb.img_size.x = 640;
  my_settings_rgb.img_size.y = 480;
  my_settings_rgb.no_segs = 400;
  my_settings_rgb.spixel_size = 24;
  my_settings_rgb.coh_weight = 0.6f;
  my_settings_rgb.no_iters = 5;
  my_settings_rgb.color_space =
      gSLICr::XYZ;  // gSLICr::CIELAB for Lab, or gSLICr::RGB for RGB
  my_settings_rgb.seg_method =
      gSLICr::GIVEN_SIZE;  // or gSLICr::GIVEN_NUM for given number
  my_settings_rgb.do_enforce_connectivity =
      true;  // whether or not run the enforce connectivity step

  const double EIGENVALUE_CUTOFF = 0.5;
  const int MAX_NUM_SEGMENTS = 30;
  const double superpixel_cue_coeff = 0.001;
  const double superpixel_smoothness_coeff = 20.0;

  // Probably always want *some* rgb distance weight, as it's possible
  // to get depth distances of 0 (and it happens pretty frequently for small
  // spider superpixels that span bad parts of the image)
  const double RGB_DISTANCE_WEIGHT = 1;
  const double DEPTH_DISTANCE_WEIGHT = 100;

  // instantiate core_engines for rgb and depth
  gSLICr::engines::core_engine* gSLICr_engine_rgb =
      new gSLICr::engines::core_engine(my_settings_rgb);

  // gSLICr takes gSLICr::UChar4Image as input and out put
  gSLICr::UChar4Image* in_img_rgb =
      new gSLICr::UChar4Image(my_settings_rgb.img_size, true, true);
  gSLICr::UChar4Image* out_img_rgb =
      new gSLICr::UChar4Image(my_settings_rgb.img_size, true, true);

  Size s_rgb(my_settings_rgb.img_size.x, my_settings_rgb.img_size.y);
  Mat frame_rgb;
  Mat boundary_draw_frame_rgb;
  boundary_draw_frame_rgb.create(s_rgb, CV_8UC3);
  Mat connectivity_img;

  StopWatchInterface* my_timer;
  sdkCreateTimer(&my_timer);

  int key;
  int save_count = 0;
  while (1) {
    auto images = camera_driver.get_latest_images();
    auto age = camera_driver.get_time_since_latest_images();

    if (age > 1.0) {
      printf("\rNo images, waiting...");
      usleep(1000 * 10);
      continue;
    }

    showGridOfImages(
        {images.first, convertToColorMap(images.second, 2000, 100)}, 2, 10,
        "input images", 640 * 3, 480);

    resize(images.first, frame_rgb, s_rgb);

    load_image(frame_rgb, in_img_rgb);

    // RGB segmentation
    sdkResetTimer(&my_timer);
    sdkStartTimer(&my_timer);
    gSLICr_engine_rgb->Process_Frame(in_img_rgb);
    sdkStopTimer(&my_timer);
    cout << "\rsegmentation of rgb frame in in:[" << sdkGetTimerValue(&my_timer)
         << "]ms\n"
         << flush;
    gSLICr_engine_rgb->Draw_Segmentation_Result(out_img_rgb);
    load_image(out_img_rgb, boundary_draw_frame_rgb);
    // Get segmentation mask and form bipartite graph
    auto label_map_rgb =
        convert_label_map_image(gSLICr_engine_rgb->Get_Seg_Res());

    imshow("segmentation", boundary_draw_frame_rgb);

    const gSLICr::IntImage* label_connectivity_img =
        gSLICr_engine_rgb->Get_Label_Connectivity();
    const gSLICr::SpixelMap* spixel_map = gSLICr_engine_rgb->Get_Spixel_Map();
    const gSLICr::objects::spixel_info* spixels =
        spixel_map->GetData(MEMORYDEVICE_CPU);

    // Show spixel connectivity map...
    connectivity_img.create(label_connectivity_img->noDims.x,
                            label_connectivity_img->noDims.y, CV_32SC1);
    load_image(label_connectivity_img, connectivity_img);

    // Start bipartite graph partitioning approach
    vector<const LabelMatrix*> label_maps;
    label_maps.push_back(&label_map_rgb);
    VectorXi num_labels_per_map(label_maps.size());
    for (int i = 0; i < label_maps.size(); i++) {
      num_labels_per_map[i] = label_maps[i]->maxCoeff() + 1;
    }

    // Sanity check all sizing lines up
    if (images.first.rows != label_maps[0]->rows() ||
        images.first.cols != label_maps[0]->cols()) {
      printf(
          "RGB image doesn't match size with first label map: %dx%d vs "
          "%ludx%lu\n",
          images.first.rows, images.first.cols, label_maps[0]->rows(),
          label_maps[0]->cols());
    }
    if (images.second.rows != label_maps[0]->rows() ||
        images.second.cols != label_maps[0]->cols()) {
      printf(
          "Depth image doesn't match size with first label map: %dx%d vs "
          "%lux%lu\n",
          images.second.rows, images.second.cols, label_maps[0]->rows(),
          label_maps[0]->cols());
    }

    int width = images.first.cols;
    int height = images.first.rows;

    // Compute statistics about depth in each spixel, rejecting the erroneous
    // zero depth
    // returns
    vector<VectorXd> min_depth_in_spixel(label_maps.size());
    vector<VectorXd> max_depth_in_spixel(label_maps.size());
    vector<VectorXd> avg_depth_in_spixel(label_maps.size());
    vector<Mat> avg_depth_in_spixel_img(label_maps.size());
    for (int label_map_i = 0; label_map_i < num_labels_per_map.rows();
         label_map_i++) {
      avg_depth_in_spixel[label_map_i] =
          VectorXd::Zero(num_labels_per_map[label_map_i]);
      min_depth_in_spixel[label_map_i] =
          VectorXd::Constant(num_labels_per_map[label_map_i],
                             std::numeric_limits<double>::infinity());
      max_depth_in_spixel[label_map_i] =
          VectorXd::Zero(num_labels_per_map[label_map_i]);
      VectorXi depth_samples_in_spixel =
          VectorXi::Zero(num_labels_per_map[label_map_i]);
      // Iterative over the depth image, finding associations of each pixel
      for (int y = 0; y < images.second.rows; y++) {
        for (int x = 0; x < images.second.cols; x++) {
          // Valid if it's nonzero (sensor returns zero for unknown areas)
          if (images.second.at<short unsigned int>(y, x) >= 100) {
            int label = label_maps[label_map_i]->coeff(y, x);
            double this_depth =
                (((double)images.second.at<short unsigned int>(y, x)) / 1000.0);
            avg_depth_in_spixel[label_map_i][label] += this_depth;
            min_depth_in_spixel[label_map_i][label] =
                fmin(min_depth_in_spixel[label_map_i][label], this_depth);
            max_depth_in_spixel[label_map_i][label] =
                fmax(max_depth_in_spixel[label_map_i][label], this_depth);
            depth_samples_in_spixel[label] += 1;
          }
        }
      }

      // Elementwise divide to get true avg depth
      for (int i = 0; i < num_labels_per_map[label_map_i]; i++) {
        if (depth_samples_in_spixel[i] != 0) {
          avg_depth_in_spixel[label_map_i][i] /=
              (double)depth_samples_in_spixel[i];
        }
      }

      // Visualize that for testing
      avg_depth_in_spixel_img[label_map_i] =
          cv::Mat::zeros(images.second.rows, images.second.cols, CV_32FC1);
      for (int y = 0; y < images.second.rows; y++) {
        for (int x = 0; x < images.second.cols; x++) {
          int label = label_maps[label_map_i]->coeff(y, x);
          avg_depth_in_spixel_img[label_map_i].at<float>(y, x) =
              avg_depth_in_spixel[label_map_i][label];
        }
      }
      avg_depth_in_spixel_img[label_map_i] =
          convertToColorMap(avg_depth_in_spixel_img[label_map_i], 2.0, 0.1);
      cout << "Avg depths by spixel for map " << label_map_i << ": "
           << avg_depth_in_spixel[label_map_i].transpose() << endl;
    }
    showGridOfImages(avg_depth_in_spixel_img,
                     ceil(sqrtf(avg_depth_in_spixel_img.size())), 10,
                     "spixels labeled by avg depth", 1280, 720);

    // Create the edge weight matrix as a sparse matrix:
    // X = layer containing the input image and then each label map in sequence
    // Y = layer containing just label maps in sequence
    printf("Starting to build weights...\n");
    SparseMatrix<double> weights(width * height + (num_labels_per_map.sum()),
                                 (num_labels_per_map.sum()));
    weights.setZero();
    // Each column (a given label in a given map) will correspond to 1 pixel
    // and some number of neighbors, no more
    weights.reserve(
        VectorXi::Constant(weights.cols(), 1 + SPIXEL_INFO_MAX_NUM_NEIGHBORS));

    printf("Building pixel superpixel affinity weights\n");
    // Bind each pixel to its corresponding coeffs
    // (assumes all pixels have a superpixel label, which I think is true
    // for all algorithms)
    for (int j = 0; j < width; j++) {
      for (int i = 0; i < height; i++) {
        int label_starting_index = 0;
        for (int label_map_i = 0; label_map_i < label_maps.size();
             label_map_i++) {
          int label = label_maps[label_map_i]->coeff(i, j);

          weights.insert(i + j * height, label_starting_index + label) =
              superpixel_cue_coeff;

          label_starting_index += num_labels_per_map[label_map_i];
        }
      }
    }

    printf("Building superpixel neighborhood weights\n");
    // Add distance function weighting for neighboring superpixels
    int label_starting_index = 0;
    for (int label_map_i = 0; label_map_i < label_maps.size(); label_map_i++) {
      for (int i = 0; i < num_labels_per_map[label_map_i]; i++) {
        bool did_any = false;
        for (int neighbor_num = 0; neighbor_num < SPIXEL_INFO_MAX_NUM_NEIGHBORS;
             neighbor_num += 1) {
          if (spixels[i].neighbors[neighbor_num] >= 0) {
            did_any = true;
            int j = spixels[i].neighbors[neighbor_num];
            if (i == j) {
              weights.insert(width * height + label_starting_index + i,
                             label_starting_index + j) = 1.0;
            } else {
              double rgb_distance = ORUtils::length(spixels[i].color_info -
                                                    spixels[j].color_info);
              // convert dpeth distances to meters
              double depth_distance_i = avg_depth_in_spixel[label_map_i][i];
              double depth_distance_j = avg_depth_in_spixel[label_map_i][j];
              double depth_distance = 0.0;
              bool depth_distance_valid = false;
              if (depth_distance_i > 0 && depth_distance_j > 0) {
                // Depth distances are so large that squaring them makes me
                // worried
                // about numerics...

                // Other idea: take shortest distance from within one
                // superpixels
                // depths to the other superpixels depths
                // This'll only work if the borders are *perfect*, otherwise
                // most neighbor distances will be zero due to fuzzy overlaps
                /*
                double depth_max_i = max_depth_in_spixel[label_map_i][i];
                double depth_min_i = min_depth_in_spixel[label_map_i][i];
                double depth_max_j = max_depth_in_spixel[label_map_i][j];
                double depth_min_j = min_depth_in_spixel[label_map_i][j];

                if (depth_max_i >= depth_min_j && depth_min_i <= depth_max_j) {
                  depth_distance = 0.0;
                } else {
                  depth_distance = fmin(powf(depth_max_i - depth_min_j, 2),
                                        powf(depth_max_j - depth_min_i, 2));
                }
                */
                depth_distance = powf(depth_distance_i - depth_distance_j, 2);
                depth_distance_valid = true;
              }

              double total_distance = 0.0;
              if (depth_distance_valid) {
                total_distance = (RGB_DISTANCE_WEIGHT * rgb_distance +
                                  DEPTH_DISTANCE_WEIGHT * depth_distance) /
                                 (RGB_DISTANCE_WEIGHT + DEPTH_DISTANCE_WEIGHT);
              } else {
                total_distance = rgb_distance;
              }

              printf("Connecitivty (%d, %d, %d) has %f, %f (%d), total %f\n",
                     label_map_i, i, j, rgb_distance, depth_distance,
                     depth_distance_valid, total_distance);

              weights.insert(width * height + label_starting_index + i,
                             label_starting_index + j) =
                  exp(-superpixel_smoothness_coeff * total_distance);
              weights.insert(width * height + label_starting_index + j,
                             label_starting_index + i) =
                  exp(-superpixel_smoothness_coeff * total_distance);
              connectivity_img.at<int>(i, j) = (int)(total_distance * 1000.);
            }
          }
        }
        if (!did_any) {
          printf("Label %d of label map %d had no neighbors???\n", i,
                 label_map_i);
        }
      }
      label_starting_index += num_labels_per_map[label_map_i];
    }
    imshow("connectivity",
           resizeKeepAspectRatio(convertToColorMap(connectivity_img, 0., 1000.),
                                 Size(500, 500), 0.5));
    /*
        // Visualize labeling per label
        vector<cv::Mat> tile_images_labels;
        for (int i = 0; i < weights.cols(); i++) {
          VectorXd subblock = weights.block(0, i, width * height, 1);
          MatrixXd this_vec =
              MatrixXd(Map<MatrixXd>(subblock.data(), height, width));
          auto im = convertToColorMap(
              matFromMatrixXd(this_vec), this_vec.minCoeff(),
       this_vec.maxCoeff());
          tile_images_labels.push_back(im);
        }
        showGridOfImages(tile_images_labels,
       ceil(sqrtf(tile_images_labels.size())), 1, "labels", 1920, 1080);
    */

    // D_x is the sum of rows of the weight matrix, inverted elementwise
    // D_y is the sum of cols of the weight matrix
    // VectorXd D_x = weights.rowwise().sum();

    printf("forming D_x and D_y\n");

    VectorXd D_x = weights * VectorXd::Ones(weights.cols());
    for (int i = 0; i < D_x.rows(); i++) {
      D_x[i] = 1. / D_x[i];
    }

    VectorXd D_y = weights.transpose() * VectorXd::Ones(weights.rows());

    printf("forming W_Y\n");

    MatrixXd W_y = weights.transpose() * D_x.asDiagonal() * weights;

    printf("forming L_Y\n");
    // We're in lower-dimensional space of # of segments (we've lost the
    // image dimension), so we'll stay in Dense matrix land from here on.
    MatrixXd L_y = MatrixXd(D_y.asDiagonal()) - W_y;

    printf("starting to compute eigs...\n");

    GeneralizedEigenSolver<MatrixXd> ges;
    ges.compute(L_y, MatrixXd(D_y.asDiagonal()));

    // Get the top N eigenvectors with largest values, taking
    // either as many as it takes to get to eigenvaluse less than
    // a threshold, or NUM_SEGMENTS of them.

    // First, set up for sorting by extracting eigenvalues into
    // a std::vector, alongside a std::vector of the indices, which
    // we'll use std::sort to sort...
    MatrixXd eigenvectors_unsorted = ges.eigenvectors().real();
    VectorXd eigenvalues_unsorted = ges.eigenvalues().real();
    printf("Sizes %d and %d, %d\n", eigenvalues_unsorted.rows(),
           eigenvectors_unsorted.rows(), eigenvectors_unsorted.cols());
    vector<pair<double, int>> eigs_and_inds(eigenvalues_unsorted.rows());
    for (int i = 0; i < eigenvalues_unsorted.rows(); i++) {
      eigs_and_inds[i].first = eigenvalues_unsorted[i];
      eigs_and_inds[i].second = i;
    }
    std::sort(eigs_and_inds.begin(), eigs_and_inds.end());

    int num_segments = min(MAX_NUM_SEGMENTS, (int)eigenvalues_unsorted.rows());
    VectorXd eigenvalues(num_segments);
    MatrixXd eigenvectors(eigenvectors_unsorted.rows(), num_segments);
    for (int i = 0; i < num_segments; i++) {
      printf("%d: %f, %d\n", i, eigs_and_inds[i].first,
             eigs_and_inds[i].second);
      eigenvalues[i] = eigs_and_inds[i].first;
      eigenvectors.col(i) = eigenvectors_unsorted.col(eigs_and_inds[i].second);
      if (eigs_and_inds[i].first > EIGENVALUE_CUTOFF) {
        eigenvalues.conservativeResize(i + 1, 1);
        eigenvectors.conservativeResize(eigenvectors.rows(), i + 1);
        break;
      }
    }

    // gizatt: from here I don't follow the math... blindly copying their
    // matlab implementation to see if it'll work

    // Transfer the eigenvectors to the entire bipartite graph
    MatrixXd eigenvectors_full = D_x.asDiagonal() * weights * eigenvectors;

    // Extractspectral representation for pixels
    eigenvectors_full.conservativeResize(width * height,
                                         eigenvectors_full.cols());

    // Visualize the eigen-images that will be clustered
    vector<cv::Mat> tile_images;
    for (int i = 0; i < eigenvectors_full.cols(); i++) {
      MatrixXd this_vec = MatrixXd(
          Map<MatrixXd>(eigenvectors_full.col(i).data(), height, width));
      this_vec.array() -= this_vec.minCoeff();
      this_vec.array() /= this_vec.maxCoeff();
      tile_images.push_back(matFromMatrixXd(this_vec));
    }
    showGridOfImages(tile_images, ceil(sqrtf(eigenvectors_full.cols())), 0,
                     "eigenvectors", 1920, 1080);

    // Normalize the rows
    eigenvectors_full.rowwise().normalize();

    // And run kmeans on that block
    int n_centers = eigenvalues.rows();
    int n_examples = eigenvectors_full.rows();
    int dimensionality = eigenvectors_full.cols();

    MatrixXd mu(n_centers, dimensionality);
    VectorXd z(n_examples);

    printf("eigenvectors full dim is %d, %d\n", eigenvectors_full.rows(),
           eigenvectors_full.cols());

    printf("entering kmeans for %d centers on %d examples of dim %d\n",
           n_centers, n_examples, dimensionality);
    RunKMeans(eigenvectors_full.data(), n_examples, dimensionality, n_centers,
              5, 42, "plusplus", mu.data(), z.data());

    cout << "Z min: " << z.minCoeff() << " and max: " << z.maxCoeff() << endl;

    // Final labels are the label assignments to each example, z:
    imshow(
        "final labels",
        convertToColorMap(
            matFromMatrixXd(MatrixXd(Map<MatrixXd>(z.data(), height, width))),
            0, n_centers, cv::COLORMAP_RAINBOW));

    key = waitKey(-1);
    if (key == 27)
      break;
    else if (key == 's') {
      char out_name[100];
      sprintf(out_name, "seg_rgb_%04i.pgm", save_count);
      gSLICr_engine_rgb->Write_Seg_Res_To_PGM(out_name);
      sprintf(out_name, "edge_rgb_%04i.png", save_count);
      imwrite(out_name, boundary_draw_frame_rgb);
      sprintf(out_name, "img_rgb_%04i.png", save_count);
      imwrite(out_name, frame_rgb);
      printf("\nsaved segmentation %04i\n", save_count);
      save_count++;
    }
  }

  destroyAllWindows();
  return 0;
}
