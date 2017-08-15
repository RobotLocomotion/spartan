/*

Given a point cloud and a template model (and configuration params),
searches for the template model in the point cloud by computing
ppf histograms at many regions of space within the point cloud, and
comparing them to the template model's ppf.

 */

#include <unistd.h>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "common/common.hpp"
#include "common/common_pcl.hpp"
#include "common/common_pcl_vtk.hpp"
#include "common/common_rtv.hpp"
#include "common/common_vtk.hpp"
#include "eigen_histogram.hpp"
#include "point_pair_features.hpp"
#include "yaml-cpp/yaml.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIdList.h>
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include <drake/math/quaternion.h>

#include "RemoteTreeViewerWrapper.hpp"

using namespace std;
using namespace Eigen;

/**
 * Computes the histogram intersection score between `source_histogram` and
 * `template_histogram` given `prior_histogram`.
 *
 */
double ComputeWeightedHistogramIntersectionScore(
    const EigenHistogram<double> &source_histogram,
    const EigenHistogram<double> &template_histogram,
    const EigenHistogram<double> &prior_histogram) {
  auto source_hist_data = source_histogram.Serialize();
  auto template_hist_data = template_histogram.Serialize();
  auto prior_hist_data = prior_histogram.Serialize();

  if (source_hist_data.size() != template_hist_data.size() ||
      source_hist_data.size() != prior_hist_data.size()) {
    printf(
        "For now, I require that all input histograms have same underlying "
        "size.");
    printf("Size conversions are still TODO(gizatt).\n");
    exit(0);
  }

  VectorXd source_vec =
      Map<VectorXi>(source_hist_data.data(), source_hist_data.size())
          .cast<double>();
  VectorXd template_vec =
      Map<VectorXi>(template_hist_data.data(), template_hist_data.size())
          .cast<double>();
  VectorXd prior_vec =
      Map<VectorXi>(prior_hist_data.data(), prior_hist_data.size())
          .cast<double>();

  // Normalize all
  source_vec.normalize();
  template_vec.normalize();
  prior_vec.normalize();

  double score = 0.0;
  for (int i = 0; i < source_hist_data.size(); i++) {
    // Yes I know the prior vec does not affect this at all...
    score += fabs( (source_vec[i] - prior_vec[i]) - (template_vec[i] - prior_vec[i]) );
  }
  return score;
}
int main(int argc, char **argv) {
  srand(0);

  if (argc != 5) {
    printf(
        "Use: run_ppf_histogram_matcher <scene cloud, vtp> "
        "<scene prior histogram, yaml>, <template model, vtp> <config>\n");
    exit(-1);
  }

  string scene_file = string(argv[1]);
  string prior_file = string(argv[2]);
  string model_file = string(argv[3]);
  string config_file = string(argv[4]);

  time_t _tm = time(NULL);
  struct tm *curtime = localtime(&_tm);
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Pair Feature Histogram Matcher" << asctime(curtime);
  cout << "Scene file " << scene_file << endl;
  cout << "Scene prior file " << prior_file << endl;
  cout << "Model file " << model_file << endl;
  cout << "***************************" << endl << endl;

  YAML::Node config = YAML::LoadFile(config_file);

  // Load in the PPF generation parameters.
  double max_distance = config["max_distance"].as<double>();
  int n_features_model = config["n_features_model"].as<int>();
  int n_features_scene_cell = config["n_features_scene_cell"].as<int>();
  int n_bins_distance = config["n_bins_distance"].as<int>();
  int n_bins_n1_n2 = config["n_bins_n1_n2"].as<int>();
  int n_bins_d_n1 = config["n_bins_d_n1"].as<int>();
  int n_bins_d_n2 = config["n_bins_d_n2"].as<int>();

  // Load in the template-matching parameters.
  double step_size = config["step_size"].as<double>();
  double minimum_pts_in_cell = config["minimum_pts_in_cell"].as<double>();

  // Load in the scene prior histogram.
  YAML::Node prior_yaml = YAML::LoadFile(prior_file);
  VectorXi n_bins_prior(4);
  n_bins_prior << prior_yaml["n_bins"]["distance"].as<double>(),
      prior_yaml["n_bins"]["n1_n2"].as<double>(),
      prior_yaml["n_bins"]["d_n1"].as<double>(),
      prior_yaml["n_bins"]["d_n2"].as<double>();
  VectorXd lb_prior(4);
  lb_prior << 0.0, 0.0, 0.0, 0.0;
  VectorXd ub_prior(4);
  ub_prior << prior_yaml["max_distance"].as<double>(), 3.1415, 3.1415, 3.1415;
  EigenHistogram<double> prior_feature_histogram(n_bins_prior, lb_prior,
                                                 ub_prior);
  prior_feature_histogram.Deserialize(
      prior_yaml["histogram"].as<vector<int>>());

  // Load the scene and model point clouds.
  Matrix<double, 6, -1> scene_point_normals =
      LoadMatrixWithVTKWithNormals(scene_file);
  Matrix<double, 6, -1> model_point_normals =
      LoadMatrixWithVTKWithNormals(model_file);

  // Visualize them.
  RemoteTreeViewerWrapper rm;
  rm.publishPointCloud(
      scene_point_normals.block(0, 0, 3, scene_point_normals.cols()),
      {"ppf_hist_matcher", "scene_pc"}, {{0.0, 0.0, 1.0}});
  rm.publishPointCloud(
      model_point_normals.block(0, 0, 3, model_point_normals.cols()),
      {"ppf_hist_matcher", "model_pc"}, {{0.0, 1.0, 0.0}});

  // Set up the paramters for histogram generation, to be shared across
  // all generated feature histograms.
  VectorXi n_bins(4);
  n_bins << n_bins_distance, n_bins_n1_n2, n_bins_d_n1, n_bins_d_n2;
  VectorXd lb_feat(4);
  lb_feat << 0.0, 0.0, 0.0, 0.0;
  VectorXd ub_feat(4);
  // Features are distance, angle between normals, and angle between
  // distance and each normal, in that order.
  ub_feat << max_distance, 3.1415, 3.1415, 3.1415, 3.1415;

  // Generate the model feature histogram, which will be shared.
  auto model_features = SamplePointPairFeatures(model_point_normals,
                                                n_features_model, max_distance);
  EigenHistogram<double> model_feature_histogram(n_bins, lb_feat, ub_feat);
  model_feature_histogram.AddData(model_features);

  printf("Generated model feature histogram. Starting search...\n");

  Vector3d scene_min_extent =
      scene_point_normals.topRows<3>().rowwise().minCoeff();
  Vector3d scene_max_extent =
      scene_point_normals.topRows<3>().rowwise().maxCoeff();
  Vector3i steps = ((scene_max_extent - scene_min_extent) / step_size)
                       .array()
                       .ceil()
                       .cast<int>();
  cout << "Min: " << scene_min_extent.transpose()
       << " and max: " << scene_max_extent.transpose() << endl;

  // Visualize the scores via heat-mapping in 3D.
  int total_n_steps = steps[0] * steps[1] * steps[2];
  Matrix3Xd sample_centroids(3, total_n_steps);
  vector<vector<double>> centroid_colors;
  centroid_colors.resize(total_n_steps, {0.0, 0.0, 0.0});
  VectorXd scores(total_n_steps);

  EigenHistogram<double> scene_cell_feature_histogram(n_bins, lb_feat, ub_feat);
  int overall_ind = 0;
  for (int i_x = 0; i_x < steps[0]; i_x++) {
    for (int i_y = 0; i_y < steps[1]; i_y++) {
      for (int i_z = 0; i_z < steps[2]; i_z++) {
        Vector3d lb_pt =
            scene_min_extent +
            Vector3d(i_x * step_size, i_y * step_size, i_z * step_size);
        Vector3d ub_pt = lb_pt + Vector3d::Ones() * step_size;

        // Extract the scene point cloud for this cell.
        Matrix<double, 6, -1> cell_point_normals(6, scene_point_normals.cols());
        int k = 0;
        for (int i = 0; i < scene_point_normals.cols(); i++) {
          auto this_pt = scene_point_normals.block<3, 1>(0, i).array();
          if ((this_pt >= lb_pt.array()).all() &&
              (this_pt <= ub_pt.array()).all()) {
            scene_point_normals.col(k) = scene_point_normals.col(i);
            k++;
          }
        }
        // Many cells will have 0, or very few, points. Reject those
        // cells, as they contain either no object or very little useful
        // signal.
        if (k < minimum_pts_in_cell) {
          continue;
        }

        // printf(
        //    "(%d, %d, %d): Application of bounds reduced point set to %d "
        //    "points\n",
        //    i_x, i_y, i_z, k);
        cell_point_normals.conservativeResize(6, k);
        auto scene_cell_features = SamplePointPairFeatures(
            model_point_normals, n_features_scene_cell, max_distance);
        scene_cell_feature_histogram.Reset();
        scene_cell_feature_histogram.AddData(scene_cell_features);

        // Compute histogram inner product.
        double score = ComputeWeightedHistogramIntersectionScore(
            scene_cell_feature_histogram, model_feature_histogram,
            prior_feature_histogram);
        // printf("\tScore: %f\n", score);

        sample_centroids.col(overall_ind) = Vector3d((lb_pt + ub_pt) / 2.);
        scores[overall_ind] = score;
        overall_ind += 1;
      }
    }
  }

  sample_centroids.conservativeResize(3, overall_ind);
  centroid_colors.resize(overall_ind);
  scores.conservativeResize(overall_ind);

  printf("Before normalization, scores ranged between min %f and max %f\n",
    scores.minCoeff(), scores.maxCoeff());
  scores -= VectorXd::Ones(scores.rows()) * scores.minCoeff();
  scores /= scores.maxCoeff();
  for (int i = 0; i < overall_ind; i++) {
    centroid_colors[i] = {powf(1. - scores[i], 2.0), powf(scores[i], 2.0), 0.0};
    cout << "Cell " << sample_centroids.col(i).transpose() << ": " << scores[i]
         << endl;
  }

  rm.publishPointCloud(sample_centroids, {"ppf_hist_matcher", "scores"},
                       centroid_colors);

  return 0;
}
