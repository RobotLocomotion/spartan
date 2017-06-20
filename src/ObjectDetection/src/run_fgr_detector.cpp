/*
 */

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "common/common.hpp"
#include "common/common_rtv.hpp"
#include "common/common_vtk.hpp"
#include "yaml-cpp/yaml.h"

#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include "RemoteTreeViewerWrapper.hpp"
#include "FastGlobalRegistration/app.h"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

typedef vector<Vector3f> Points;
typedef vector<VectorXf> Feature;

const Vector3d doNormalConsensusWithVector3d(pcl::PointNormal& pt, const Vector3d desired_normal){
  Vector3d pt_normal = Map<const Vector3f>(pt.normal).cast<double>();
  if (desired_normal.transpose() * pt_normal < 0){
    for (int i = 0; i < 3; i++) {
      pt.normal[i] *= -1.;
    }
  }
  return Map<const Vector3f>(pt.normal).cast<double>();
}

struct Node {
  std::vector<int> outgoing_edges_inds;
  std::vector<float> outgoing_edges_weights;
  bool fixed;
};

std::pair<Points, Feature> generatePointsAndFeaturesFromPoints(Eigen::Matrix3Xd input_points, std::vector<std::string> prefix = {}, bool orient_normals = false){

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  Vector3d bb_min = input_points.rowwise().minCoeff();
  Vector3d bb_max = input_points.rowwise().maxCoeff();
  double diag = (bb_max - bb_min).norm();
  double interpoint_scale = (diag / powf( (double) input_points.size(), 0.33)) * sqrtf(3);
  for (int i = 0; i < input_points.cols(); i++) {
    pcl::PointXYZ temp;
    temp.x = input_points(0, i);
    temp.y = input_points(1, i);
    temp.z = input_points(2, i);
    input_cloud->push_back(temp);
  }

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (input_cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use 10 neighbors
  ne.setKSearch (10);
  printf("Using diag %f and interpoint scale %f\n", diag, interpoint_scale);
  ne.compute (*cloud_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
  pcl::concatenateFields(*input_cloud, *cloud_normals, *input_cloud_with_normals); 

  if (orient_normals) {
    printf("Unifying normals...\n");

    // Make normal edge directions consistent.
    // Form a tree of all points, with edges connecting points within a given distance.
    std::vector<Node> graph;

    for (int i = 0; i < input_points.size(); i++) {
      Node new_node;
      new_node.fixed = false;
      tree->radiusSearch (input_cloud->points[i], interpoint_scale, new_node.outgoing_edges_inds, new_node.outgoing_edges_weights);
      graph.push_back(new_node);
    }
    
    int next;
    input_points.row(2).maxCoeff(&next);
    auto this_norm = doNormalConsensusWithVector3d(input_cloud_with_normals->points[next], {0.0, 0.0, 1.0});
    graph[next].fixed = true;
    std::deque<pair<int, Vector3d>> next_queue; for (const auto i : graph[next].outgoing_edges_inds) next_queue.push_back({i, this_norm});
    while (next_queue.size() > 0){
      next = next_queue[0].first;
      auto last_norm = next_queue[0].second;
      next_queue.pop_front();

      if (graph[next].fixed)
        continue;

      this_norm = doNormalConsensusWithVector3d(input_cloud_with_normals->points[next], last_norm);
      graph[next].fixed = true;
      for (const auto i : graph[next].outgoing_edges_inds){
        if (!graph[i].fixed){
     //     printf("Adding %d, which has fixed %d\n", i, graph[i].fixed);
          next_queue.push_back({i, this_norm});
        }
      }
     // printf("Queue size: %lu\n", next_queue.size());
      usleep(10);
    }
    printf("Done\n");
  }


  // For all model and scene points, generate FPFH features
  // (this code is modified from https://github.com/IntelVCL/FastGlobalRegistration)
  pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
  fest.setRadiusSearch(interpoint_scale*4);  
  fest.setInputCloud(input_cloud_with_normals);
  fest.setInputNormals(input_cloud_with_normals);
  fest.compute(*object_features);

  RemoteTreeViewerWrapper rm;
  std::vector<string> label;
  label.insert(label.end(), prefix.begin(), prefix.end());
  label.push_back("pointsandfeatures");
  label.push_back("points");
  rm.publishPointCloud(input_points, label);

  Points output_points;
  Feature output_features;
  for (int i = 0; i < input_points.cols(); i++) {
    const pcl::PointNormal &pt = input_cloud_with_normals->points[i];
    output_points.push_back({pt.x, pt.y, pt.z});
    const pcl::FPFHSignature33 &feature = object_features->points[i];
    output_features.push_back(Map<const VectorXf>(feature.histogram, 33));

    if (i % (input_points.cols() / 100) == 0) {
      Vector3d normal = Map<const Vector3f>(pt.normal).cast<double>();
      Matrix<double, 3, 2> normal_line;
      normal_line.col(0) = input_points.col(i);
      normal_line.col(1) = input_points.col(i) + normal*0.05;
      char buf[20]; sprintf(buf, "normal_%d", i);
      label.clear();  
      label.insert(label.end(), prefix.begin(), prefix.end());
      label.push_back("pointsandfeatures");
      label.push_back(buf);
      rm.publishLine(normal_line, label);
    }
  }
  return {output_points, output_features};
}

int main(int argc, char** argv) {
  srand(0);

  if (argc < 3){
    printf("Use: run_fgr_detector <scene cloud, vtp> <model cloud, vtp> <config file> <optional output_file>\n");
    exit(-1);
  }

  string sceneFile = string(argv[1]);
  string modelFile = string(argv[2]);
  string configFile = string(argv[3]);
  string outputFile = argc > 3 ? string(argv[4]) : "";

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Fast Global Registration Object Pose Estimator" << asctime(curtime);
  cout << "Scene file " << sceneFile << endl;
  cout << "Model file " << modelFile << endl;
  cout << "Config file " << configFile << endl;
  if (argc > 3)
    cout << "Output file " << outputFile << endl;
  cout << "***************************" << endl << endl;

  // Bring in config file
  YAML::Node config = YAML::LoadFile(configFile);

  if (config["detector_options"] == NULL){
    runtime_error("Need detector options.");
  }
  auto fgr_config = config["detector_options"];

  // Read in the model.
  vtkSmartPointer<vtkPolyData> modelPolyData = ReadPolyData(modelFile.c_str());
  cout << "Loaded " << modelPolyData->GetNumberOfPoints() << " points from " << modelFile << endl;
  Matrix3Xd model_pts(3, modelPolyData->GetNumberOfPoints());
  for (int i=0; i<modelPolyData->GetNumberOfPoints(); i++){
    model_pts(0, i) = modelPolyData->GetPoint(i)[0];
    model_pts(1, i) = modelPolyData->GetPoint(i)[1];
    model_pts(2, i) = modelPolyData->GetPoint(i)[2];
  }

  // Load in the scene cloud
  vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(sceneFile.c_str());
  cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from " << sceneFile << endl;
  Matrix3Xd scene_pts(3, cloudPolyData->GetNumberOfPoints());
  for (int i=0; i<cloudPolyData->GetNumberOfPoints(); i++){
    scene_pts(0, i) = cloudPolyData->GetPoint(i)[0];
    scene_pts(1, i) = cloudPolyData->GetPoint(i)[1];
    scene_pts(2, i) = cloudPolyData->GetPoint(i)[2];
  }

  // Visualize the scene points and GT, to start with.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  //rm.publishPointCloud(scene_pts_in, {"scene_pts_loaded"}, {{0.1, 1.0, 0.1}});
  rm.publishPointCloud(scene_pts, {"fgr", "scene_pts_downsampled"}, {{0.1, 1.0, 1.0}});
  //rm.publishPointCloud(model_pts_in, {"model_pts"}, {{0.1, 1.0, 1.0}});
  rm.publishPointCloud(model_pts, {"fgr", "model_pts_downsampled"}, {{0.1, 1.0, 1.0}});
  //rm.publishRigidBodyTree(robot, q_robot, Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});


  // Load in the FGR estimator
  CApp fgr_app;

  cout << "Generating features and matching..." << endl;
  clock_t clockBegin = clock();
  
  auto model_info = generatePointsAndFeaturesFromPoints(model_pts, {"fgr", "model"}, true);
  auto scene_info = generatePointsAndFeaturesFromPoints(scene_pts, {"fgr", "scene"});

  printf("Adding %lu model points and %lu model features\n", model_info.first.size(), model_info.second.size());
  fgr_app.LoadFeature(model_info.first, model_info.second); 

  printf("Adding %lu scene points and %lu scene features\n", scene_info.first.size(), scene_info.second.size());
  fgr_app.LoadFeature(scene_info.first, scene_info.second); 

  fgr_app.NormalizePoints();
  fgr_app.AdvancedMatching();
  fgr_app.OptimizePairwise(true, ITERATION_NUMBER); // defaults to 64 iterations, see FastGlobalRegistration/app.h
  
  clock_t clockEnd = clock();
  double time = (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC;
  cout << "Finished in " << time << endl;
  Affine3d est_tf;
  est_tf.matrix() = fgr_app.GetTrans().cast<double>();
 // est_tf = est_tf.inverse();

  // Publish the transformed scene point cloud (I'm transforming the scene because
  // the model is usually better centered )
  cout << "Est tf " << est_tf.matrix() << endl;
  cout << "Est tf inv" << est_tf.inverse().matrix() << endl;

  publishErrorColorCodedPointCloud(est_tf * scene_pts, model_pts, "fgr");

  return 0;
}
