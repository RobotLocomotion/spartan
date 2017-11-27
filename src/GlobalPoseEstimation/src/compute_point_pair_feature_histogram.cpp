/*
  Generates point pair feature histograms from a model.

  Configuration supports several options:
    - Default mode of operation generates ppfs sampled from
      the entire point cloud, with point pairs generated
      according to the rules of the config, and spits them
      out to `output_file.`
    - Restricted-range mode of operation generates ppfs from
      a restricted domain of the point cloud, and spits them
      out to `output_file.`
    - Template-matching-prep mode generates a ppf given a stepping
      pattern (similar to that used in the ppf histogram matcher),
      and spits them out to files in the `output_file` subdirectory.
      This can be used to collect statistics about the histograms
      that the histogram matcher sees on its template matching step.
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

void SaveFeatureHistogram(const EigenHistogram<double>& feature_histogram,
                          const std::string& output_file, int n_features,
                          double max_distance, int n_bins_distance,
                          int n_bins_n1_n2, int n_bins_d_n1, int n_bins_d_n2) {
  YAML::Emitter out;

  out << YAML::BeginMap;
  {
    out << YAML::Key << "n_features";
    out << YAML::Value << n_features;

    out << YAML::Key << "max_distance";
    out << YAML::Value << max_distance;

    out << YAML::Key << "n_bins";
    out << YAML::BeginMap;
    {
      out << YAML::Key << "distance";
      out << YAML::Value << n_bins_distance;
      out << YAML::Key << "n1_n2";
      out << YAML::Value << n_bins_n1_n2;
      out << YAML::Key << "d_n1";
      out << YAML::Value << n_bins_d_n1;
      out << YAML::Key << "d_n2";
      out << YAML::Value << n_bins_d_n2;
    }
    out << YAML::EndMap;

    out << YAML::Key << "histogram";
    out << YAML::Value << YAML::Flow << feature_histogram.Serialize();
  }
  out << YAML::EndMap;

  ofstream fout(output_file);
  fout << out.c_str();
  fout.close();
}

int main(int argc, char** argv) {
  srand(0);

  if (argc != 4) {
    printf(
        "Use: compute_point_pair_feature_histogram <scene cloud, vtp> "
        "<output_file> <config>\n");
    exit(-1);
  }

  string scene_file = string(argv[1]);
  string output_file = string(argv[2]);
  string config_file = string(argv[3]);

  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Pair Feature Generator" << endl;
  cout << "Scene file " << scene_file << endl;
  cout << "Output file " << output_file << endl;
  cout << "***************************" << endl << endl;

  YAML::Node config = YAML::LoadFile(config_file);

  double max_distance = config["max_distance"].as<double>();
  int n_features = config["n_features"].as<int>();
  int n_bins_distance = config["n_bins_distance"].as<int>();
  int n_bins_n1_n2 = config["n_bins_n1_n2"].as<int>();
  int n_bins_d_n1 = config["n_bins_d_n1"].as<int>();
  int n_bins_d_n2 = config["n_bins_d_n2"].as<int>();

  // Load in the template-matching parameters.
  double step_size = 0.0;
  double minimum_pts_in_cell = 0;
  if (config["step_size"] && config["minimum_pts_in_cell"]) {
    step_size = config["step_size"].as<double>();
    minimum_pts_in_cell = config["minimum_pts_in_cell"].as<double>();
  }

  // Load in bound-restriction parameters
  bool have_bounds = false;
  Vector3d lb_pt, ub_pt;
  if (config["bounds_center"] && config["bounds_width"]) {
    vector<double> b_center_vector =
        config["bounds_center"].as<vector<double>>();
    Vector3d b_center(b_center_vector[0], b_center_vector[1],
                      b_center_vector[2]);
    double b_width = config["bounds_width"].as<double>();
    lb_pt = b_center - Vector3d::Ones() * b_width / 2.;
    ub_pt = b_center + Vector3d::Ones() * b_width / 2.;
    have_bounds = true;
    cout << "Bounds: lb: " << lb_pt.transpose() << ", ub: " << ub_pt.transpose()
         << endl;
  }

  Matrix<double, 6, -1> scene_point_normals =
      LoadMatrixWithVTKWithNormals(scene_file);

  if (have_bounds) {
    Matrix<double, 6, -1> scene_point_normalsReduced(
        6, scene_point_normals.cols());
    int k = 0;
    for (int i = 0; i < scene_point_normals.cols(); i++) {
      auto this_pt = scene_point_normals.block<3, 1>(0, i).array();
      if ((this_pt >= lb_pt.array()).all() &&
          (this_pt <= ub_pt.array()).all()) {
        scene_point_normalsReduced.col(k) = scene_point_normals.col(i);
        k++;
      }
    }
    printf("Application of bounds reduced point set to %d points\n", k);
    scene_point_normalsReduced.conservativeResize(6, k);
    scene_point_normals = scene_point_normalsReduced;
  }

  RemoteTreeViewerWrapper rm;
  rm.publishPointCloud(
      scene_point_normals.block(0, 0, 3, scene_point_normals.cols()),
      {"scene_point_normals"});

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

  EigenHistogram<double> feature_histogram(n_bins, lb_feat, ub_feat);

  if (step_size > 0.0) {
    // Ugh, making directories from C... TODO(gizatt) do this not-so-sketchily.
    char buf[100];
    sprintf(buf, "mkdir -p %s", output_file.c_str());
    int err = system(buf);

    Vector3d scene_min_extent =
        scene_point_normals.topRows<3>().rowwise().minCoeff();
    Vector3d scene_max_extent =
        scene_point_normals.topRows<3>().rowwise().maxCoeff();
    Vector3i steps = ((scene_max_extent - scene_min_extent) / step_size)
                         .array()
                         .ceil()
                         .cast<int>();

    int total_n_steps = steps[0] * steps[1] * steps[2];
    int overall_ind = 0;
    for (int i_x = 0; i_x < steps[0]; i_x++) {
      for (int i_y = 0; i_y < steps[1]; i_y++) {
        for (int i_z = 0; i_z < steps[2]; i_z++) {
          overall_ind++;
          if (overall_ind % (total_n_steps / 20) == 0) {
            printf("%d/%d done.\n", overall_ind, total_n_steps);
          }
          Vector3d lb_pt =
              scene_min_extent +
              Vector3d(i_x * step_size, i_y * step_size, i_z * step_size);
          Vector3d ub_pt = lb_pt + Vector3d::Ones() * step_size;

          // Extract the scene point cloud for this cell.
          Matrix<double, 6, -1> cell_point_normals(6,
                                                   scene_point_normals.cols());
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

          cell_point_normals.conservativeResize(6, k);
          auto scene_cell_features = SamplePointPairFeatures(
              scene_point_normals, n_features, max_distance);
          feature_histogram.Reset();
          feature_histogram.AddData(scene_cell_features);
          sprintf(buf, "%s/%04d_%04d_%04d.ppfh", output_file.c_str(), i_x, i_y,
                  i_z);
          SaveFeatureHistogram(feature_histogram, buf, n_features, max_distance,
                               n_bins_distance, n_bins_n1_n2, n_bins_d_n1,
                               n_bins_d_n2);
        }
      }
    }
  } else {
    // Generate point-pair features for whole remaining point cloud.
    auto features =
        SamplePointPairFeatures(scene_point_normals, n_features, max_distance);
    feature_histogram.AddData(features);
    SaveFeatureHistogram(feature_histogram, output_file, n_features,
                         max_distance, n_bins_distance, n_bins_n1_n2,
                         n_bins_d_n1, n_bins_d_n2);
  }

  printf("Done!\n");
  return 0;
}
