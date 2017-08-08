/*
 */

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "common/common.hpp"
#include "common/common_pcl.hpp"
#include "common/common_pcl_vtk.hpp"
#include "common/common_rtv.hpp"
#include "common/common_vtk.hpp"
#include "point_pair_features.hpp"
#include "eigen_histogram.hpp"
#include "yaml-cpp/yaml.h"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIdList.h>
#include <vtkIntArray.h>
#include <vtkObjectFactory.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include <drake/math/quaternion.h>

#include "RemoteTreeViewerWrapper.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  srand(0);

  if (argc != 4){
    printf("Use: compute_point_pair_feature_histogram <scene cloud, vtp> <output_file> <config>\n");
    exit(-1);
  }

  string sceneFile = string(argv[1]);
  string outputFile = string(argv[2]);
  string configFile = string(argv[3]);

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Pair Feature Generator" << asctime(curtime);
  cout << "Scene file " << sceneFile << endl;
  cout << "Output file " << outputFile << endl;
  cout << "***************************" << endl << endl;

  YAML::Node config = YAML::LoadFile(configFile);

  double max_distance = config["max_distance"].as<double>();
  int n_features = config["n_features"].as<int>();
  int n_bins_distance = config["n_bins_distance"].as<int>();
  int n_bins_n1_n2 = config["n_bins_n1_n2"].as<int>();
  int n_bins_d_n1 = config["n_bins_d_n1"].as<int>();
  int n_bins_d_n2 = config["n_bins_d_n2"].as<int>();

  bool have_bounds = false;
  Vector3d lb_pt, ub_pt;
  if (config["bounds_center"] && config["bounds_width"]) {
    vector<double> b_center_vector = config["bounds_center"].as<vector<double>>();
    Vector3d b_center(b_center_vector[0], b_center_vector[1], b_center_vector[2]);
    double b_width = config["bounds_width"].as<double>();
    lb_pt = b_center - Vector3d::Ones()*b_width/2.;
    ub_pt = b_center + Vector3d::Ones()*b_width/2.;
    have_bounds = true;
  }

  Matrix<double, 6, -1> pointNormals = LoadMatrixWithVTKWithNormals(sceneFile);

  if (have_bounds) {
    Matrix<double, 6, -1> pointNormalsReduced(6, pointNormals.cols());
    int k = 0;
    for (int i = 0; i < pointNormals.cols(); i++) {
      auto this_pt = pointNormals.block<3, 1>(0, i).array();
      if ( (this_pt >= lb_pt.array()).all() && (this_pt <= ub_pt.array()).all() ) {
        pointNormalsReduced.col(i) = pointNormals.col(i);
        k++;
      }
    }
    printf("Application of bounds reduced point set to %d points\n", k);
    pointNormalsReduced.conservativeResize(6, k);
    pointNormals = pointNormalsReduced;
  }

  // Generate point-pair features
  auto features = SamplePointPairFeatures(pointNormals, n_features, max_distance);

  VectorXi n_bins(4);
  n_bins << n_bins_distance, n_bins_n1_n2, n_bins_d_n1, n_bins_d_n2;
  VectorXd lb_feat(4);
  lb_feat << 0.0, 0.0, 0.0, 0.0;
  VectorXd ub_feat(4);
  ub_feat << max_distance, 3.1415, 3.1415, 3.1415, 3.1415;
  EigenHistogram<double> feature_histogram(n_bins, lb_feat, ub_feat);
  feature_histogram.AddData(features);

  YAML::Emitter out;

  out << YAML::BeginMap; {
    out << YAML::Key << "n_features";
    out << YAML::Value << n_features;

    out << YAML::Key << "max_distance";
    out << YAML::Value << max_distance;

    out << YAML::Key << "n_bins";
    out << YAML::BeginMap; {
      out << YAML::Key << "distance";
      out << YAML::Value << n_bins_distance;
      out << YAML::Key << "n1_n2";
      out << YAML::Value << n_bins_n1_n2;
      out << YAML::Key << "d_n1";
      out << YAML::Value << n_bins_d_n1;
      out << YAML::Key << "d_n2";
      out << YAML::Value << n_bins_d_n2;
    } out << YAML::EndMap;

    out << YAML::Key << "histogram";
    out << YAML::Value << YAML::Flow << feature_histogram.Serialize();
    } out << YAML::EndMap;

    ofstream fout(outputFile);
    fout << out.c_str();
    fout.close();

    printf("Done!\n");
  return 0;
}
