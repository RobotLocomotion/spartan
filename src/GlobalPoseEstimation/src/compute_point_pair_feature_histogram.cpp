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

  if (argc != 9){
    printf("Use: compute_point_pair_feature_histogram <scene cloud, vtp> <output_file> <# features> <# distance bins> <max distance> <# n1_n2 bins> <# d_n1 bins> <# d_n2 bins>\n");
    exit(-1);
  }

  string sceneFile = string(argv[1]);
  string outputFile = string(argv[2]);
  int n_features = atoi(argv[3]);
  int n_bins_distance = atoi(argv[4]);
  double max_distance = atof(argv[5]);
  int n_bins_n1_n2 = atoi(argv[6]);
  int n_bins_d_n1 = atoi(argv[7]);
  int n_bins_d_n2 = atoi(argv[8]);

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Pair Feature Generator" << asctime(curtime);
  cout << "Scene file " << sceneFile << endl;
  cout << "Output file " << outputFile << endl;
  cout << "***************************" << endl << endl;

  auto pointNormals = LoadMatrixWithVTKWithNormals(sceneFile);

  // Generate point-pair features
  auto features = SamplePointPairFeatures(pointNormals, n_features, max_distance);

  VectorXi n_bins(4);
  n_bins << n_bins_distance, n_bins_n1_n2, n_bins_d_n1, n_bins_d_n2;
  VectorXd lb(4);
  lb << 0.0, 0.0, 0.0, 0.0;
  VectorXd ub(4);
  ub << max_distance, 3.1415, 3.1415, 3.1415, 3.1415;
  EigenHistogram<double> feature_histogram(n_bins, lb, ub);
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

  return 0;
}
