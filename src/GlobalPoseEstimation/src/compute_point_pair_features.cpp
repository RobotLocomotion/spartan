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

typedef vector<Vector3f> Points;
typedef vector<VectorXf> Feature;

int main(int argc, char** argv) {
  srand(0);

  if (argc != 5){
    printf("Use: compute_point_pair_features <scene cloud, vtp> <output_file> <# features> <with_vis?>\n");
    exit(-1);
  }

  string sceneFile = string(argv[1]);
  string outputFile = string(argv[2]);
  int n_features = atoi(argv[3]);
  int with_vis = atoi(argv[4]);

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Pair Feature Generator" << asctime(curtime);
  cout << "Scene file " << sceneFile << endl;
  cout << "Output file " << outputFile << endl;
  cout << "Generating " << n_features << " features" << endl;
  cout << "***************************" << endl << endl;

  // Load in the scene cloud
  vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(sceneFile.c_str());
  cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from " << sceneFile << endl;

  pcl::PointCloud<pcl::PointNormal>::Ptr sceneCloud(new pcl::PointCloud<pcl::PointNormal>());
  if (cloudPolyData->GetPointData()->GetNormals()) {
    sceneCloud = PointCloudFromPolyDataWithNormals(cloudPolyData);
  } else {
    printf("No normals available in input scene file... generating own.\n");
    printf("Note: it's possible that VTK just isn't loading your normals!\n");
    printf("The STL reader is known not to read normals. Try using meshlab to convert to OBJ.\n");
    printf("Regenerate your vertex normals while you're at it :)\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudWithoutNormals =
        PointCloudFromPolyData(cloudPolyData);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sceneCloudWithoutNormals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr sceneCloudNormals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.setViewPoint (0.0, 0.0, 0.0);
    // Compute the features
    ne.compute (*sceneCloudNormals);

    assert(sceneCloudWithoutNormals->size() == sceneCloudNormals->size());
    sceneCloud->points.resize(sceneCloudWithoutNormals->size());
    printf("Merging clouds...\n");
    for (size_t i = 0; i < sceneCloud->points.size(); i++) {
        sceneCloud->points[i].x = sceneCloudWithoutNormals->points[i].x;
        sceneCloud->points[i].y = sceneCloudWithoutNormals->points[i].y;
        sceneCloud->points[i].z = sceneCloudWithoutNormals->points[i].z;
        sceneCloud->points[i].normal_x = sceneCloudNormals->points[i].normal_x;
        sceneCloud->points[i].normal_y = sceneCloudNormals->points[i].normal_y;
        sceneCloud->points[i].normal_z = sceneCloudNormals->points[i].normal_z;
    }
  }
  printf("Done and loaded.\n");


  auto pointNormals = convertPCLPointNormalToMatrix6Xd(sceneCloud);

  if (with_vis) {
    vector<string> prefix = {"ppf"};
    vector<string> label;
    RemoteTreeViewerWrapper rm;
    if (with_vis) {
      label.insert(label.end(), prefix.begin(), prefix.end());
      label.push_back("pointsandfeatures");
      label.push_back("points");
      rm.publishPointCloud(pointNormals.block(0, 0, 3, pointNormals.cols()), label);
    }

    // # pts drawn = pointNormals.cols() / mod_ratio_used
    // mod_ratio_used = pointNormals.cols() / # pts drawn
    int pt_ratio_to_use = pointNormals.cols() / 250;
    if (pt_ratio_to_use < 1) pt_ratio_to_use = 1;
    for (int i = 0; i < pointNormals.cols(); i++) {
      if (with_vis && i % pt_ratio_to_use == 0) {
        Vector3d normal = pointNormals.block<3, 1>(3, i);
        Matrix<double, 3, 2> normal_line;
        normal_line.col(0) = pointNormals.block<3, 1>(0, i);;
        normal_line.col(1) = pointNormals.block<3, 1>(0, i) + normal*0.05;
        char buf[20]; sprintf(buf, "normal_%d", i);
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

  ofstream outfile;
  outfile.open (outputFile);

  int n_pts = sceneCloud->size();
  for (int i = 0; i < n_features; i++) {
    int i_1 = rand() % n_pts;
    int i_2 = i_1;
    while (i_1 == i_2)
      i_2 = rand() % n_pts;

    Vector3d d = pointNormals.block<3, 1>(0, i_1) - pointNormals.block<3, 1>(0, i_2);
    double dist = d.norm();
    d /= d.norm();
    Vector3d n_1 = pointNormals.block<3, 1>(3, i_1);
    n_1 /= n_1.norm();
    Vector3d n_2 = pointNormals.block<3, 1>(3, i_2);
    n_2 /= n_2.norm();

    outfile << i_1 << ", ";
    outfile << i_2 << ", ";
    outfile << dist << ", ";
    outfile << calculateAngleBetweenUnitVectors(n_1, n_2) << ", ";
    outfile << calculateAngleBetweenUnitVectors(d, n_1) << ", ";
    outfile << calculateAngleBetweenUnitVectors(d, n_2);
    outfile << endl;
  }

  outfile.close();


  // Save them out

  return 0;
}
