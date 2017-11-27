/*
 */

#include <unistd.h>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "common_utils/math_utils.h"
#include "common_utils/pcl_utils.h"
#include "common_utils/pcl_vtk_utils.h"
#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"
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
#include "Super4PCS/src/super4pcs/shared4pcs.h"

using namespace std;
using namespace Eigen;
using namespace match_4pcs;

typedef vector<Vector3f> Points;
typedef vector<VectorXf> Feature;

int main(int argc, char** argv) {
  srand(0);

  if (argc != 12) {
    printf(
        "Use: crop_pointcloud_cloud_to_model <scene cloud, vtp> <model file, "
        "vtp> <output_file> <crop_distance> <model_x> <model_y> <model_z> "
        "<model_qw> <model_qx> <model_qy> <model_qz>\n");
    exit(-1);
  }

  string sceneFile = string(argv[1]);
  string modelFile = string(argv[2]);
  string outputFile = string(argv[3]);

  double crop_distance = atof(argv[4]);
  VectorXd q0(7);
  q0 << atof(argv[5]), atof(argv[6]), atof(argv[7]), atof(argv[8]),
      atof(argv[9]), atof(argv[10]), atof(argv[11]);

  time_t _tm = time(NULL);
  struct tm* curtime = localtime(&_tm);
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Pointcloud Cropper" << asctime(curtime);
  cout << "Scene file " << sceneFile << endl;
  cout << "Model file " << modelFile << endl;
  cout << "Output file " << outputFile << endl;
  cout << "Cropping to dist " << crop_distance << " and model pose "
       << q0.transpose() << endl;
  cout << "***************************" << endl << endl;

  // Read in the model.
  vtkSmartPointer<vtkPolyData> modelPolyData = ReadPolyData(modelFile.c_str());
  cout << "Loaded " << modelPolyData->GetNumberOfPoints() << " points from "
       << modelFile << endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud =
      PointCloudFromPolyData(modelPolyData);

  // build tf
  Affine3d tf;
  tf.setIdentity();
  tf.translation() = q0.block<3, 1>(0, 0);
  tf.matrix().block<3, 3>(0, 0) =
      drake::math::quat2rotmat(q0.block<4, 1>(3, 0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudTF(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*modelCloud, *modelCloudTF, tf.cast<float>());

  // Load in the scene cloud
  vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(sceneFile.c_str());
  cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from "
       << sceneFile << endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud =
      PointCloudFromPolyData(cloudPolyData);

  // Assemble output cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr outSceneCloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(modelCloudTF);

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  int nr_points = sceneCloud->points.size();
  for (int i = 0; i < nr_points; i++) {
    if (i % (nr_points / 10) == 0) {
      printf("%d / %d\n", i, nr_points);
    }
    pcl::PointXYZ searchPoint = sceneCloud->points[i];
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                              pointNKNSquaredDistance) > 0) {
      double dist = sqrtf(pointNKNSquaredDistance[0]);
      if (dist < crop_distance) {
        outSceneCloud->push_back(searchPoint);
      }
    }
  }

  // Convert back to polydata
  vtkSmartPointer<vtkPolyData> outPolyData =
      PolyDataFromPointCloud(outSceneCloud);
  // outPolyData->SetPoints(outPoints);
  WritePolyData(outPolyData, outputFile.c_str());

  return 0;
}
