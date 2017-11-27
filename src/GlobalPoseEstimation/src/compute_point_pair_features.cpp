/*
 */

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <unistd.h>

#include "common_utils/math_utils.h"
#include "common_utils/pcl_utils.h"
#include "common_utils/pcl_vtk_utils.h"
#include "common_utils/vtk_utils.h"
#include "common_utils/eigen_histogram.h"
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

int main(int argc, char **argv) {
  srand(0);

  if (argc != 6) {
    printf("Use: compute_point_pair_features <scene cloud, vtp> <output_file> "
           "<# features> <with_vis?> <max distance>\n");
    exit(-1);
  }

  string sceneFile = string(argv[1]);
  string outputFile = string(argv[2]);
  int n_features = atoi(argv[3]);
  int with_vis = atoi(argv[4]);
  double max_distance = atoi(argv[5]);

  time_t _tm = time(NULL);
  struct tm *curtime = localtime(&_tm);
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Pair Feature Generator" << asctime(curtime);
  cout << "Scene file " << sceneFile << endl;
  cout << "Output file " << outputFile << endl;
  cout << "Generating " << n_features << " features" << endl;
  cout << "***************************" << endl << endl;

  auto pointNormals = LoadMatrixWithVTKWithNormals(sceneFile);

  if (with_vis) {
    vector<string> prefix = {"ppf"};
    vector<string> label;
    RemoteTreeViewerWrapper rm;
    if (with_vis) {
      label.insert(label.end(), prefix.begin(), prefix.end());
      label.push_back("pointsandfeatures");
      label.push_back("points");
      rm.publishPointCloud(pointNormals.block(0, 0, 3, pointNormals.cols()),
                           label);
    }

    // # pts drawn = pointNormals.cols() / mod_ratio_used
    // mod_ratio_used = pointNormals.cols() / # pts drawn
    int pt_ratio_to_use = pointNormals.cols() / 250;
    if (pt_ratio_to_use < 1)
      pt_ratio_to_use = 1;
    for (int i = 0; i < pointNormals.cols(); i++) {
      if (with_vis && i % pt_ratio_to_use == 0) {
        Vector3d normal = pointNormals.block<3, 1>(3, i);
        Matrix<double, 3, 2> normal_line;
        normal_line.col(0) = pointNormals.block<3, 1>(0, i);
        ;
        normal_line.col(1) = pointNormals.block<3, 1>(0, i) + normal * 0.05;
        char buf[20];
        sprintf(buf, "normal_%d", i);
        label.clear();
        label.insert(label.end(), prefix.begin(), prefix.end());
        label.push_back("pointsandfeatures");
        label.push_back("normals");
        label.push_back(buf);
        rm.publishLine(normal_line, label);
      }
    }
  }

  // Generate point-pair features and save them
  auto features =
      SamplePointPairFeatures(pointNormals, n_features, max_distance);

  ofstream outfile;
  outfile.open(outputFile);

  for (int i = 0; i < features.cols(); i++) {
    outfile << features(i, 0) << ", ";
    outfile << features(i, 1) << ", ";
    outfile << features(i, 2) << ", ";
    outfile << features(i, 3);
    outfile << endl;
  }
  outfile.close();

  // Save them out

  return 0;
}
