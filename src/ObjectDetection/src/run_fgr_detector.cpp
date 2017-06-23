/*
 */

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "common/common.hpp"
#include "common/common_pcl.hpp"
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


  // Load in the FGR estimator and its options.
  CApp fgr_app;
  if (fgr_config["div_factor"])
    fgr_app.div_factor_ = fgr_config["div_factor"].as<double>();
  if (fgr_config["use_absolute_scale"])
    fgr_app.use_absolute_scale_ = fgr_config["use_absolute_scale"].as<bool>();
  if (fgr_config["max_corr_dist"])
    fgr_app.max_corr_dist_ = fgr_config["max_corr_dist"].as<double>();
  if (fgr_config["iteration_number"])
    fgr_app.iteration_number_ = fgr_config["iteration_number"].as<int>();
  if (fgr_config["tuple_scale"])
    fgr_app.tuple_scale_ = fgr_config["tuple_scale"].as<double>();
  if (fgr_config["tuple_max_cnt"])
    fgr_app.tuple_max_cnt_ = fgr_config["tuple_max_cnt"].as<double>();

  double interpoint_scale = -1;
  if (fgr_config["interpoint_scale"])
    interpoint_scale = fgr_config["interpoint_scale"].as<double>();
  double feature_radius_multiplier = 10.0;
  if (fgr_config["feature_radius_multiplier"])
    feature_radius_multiplier = fgr_config["feature_radius_multiplier"].as<double>();

  cout << "Generating features and matching..." << endl;
  clock_t clockBegin = clock();
  
  auto model_info = generatePointsAndFPFHFeaturesFromPoints(model_pts, {"fgr", "model"}, true, true, interpoint_scale, feature_radius_multiplier);
  auto scene_info = generatePointsAndFPFHFeaturesFromPoints(scene_pts, {"fgr", "scene"}, false, true, interpoint_scale, feature_radius_multiplier);


  printf("Adding %lu model points and %lu model features\n", model_info.first.size(), model_info.second.size());
  fgr_app.LoadFeature(model_info.first, model_info.second); 

  printf("Adding %lu scene points and %lu scene features\n", scene_info.first.size(), scene_info.second.size());
  fgr_app.LoadFeature(scene_info.first, scene_info.second); 

  fgr_app.NormalizePoints();
  fgr_app.AdvancedMatching();
  fgr_app.OptimizePairwise(true, fgr_app.iteration_number_);
  
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

  VectorXd q_out(7);
  q_out.block<3, 1>(0, 0) = est_tf.inverse().translation();
  q_out.block<4, 1>(3, 0) = drake::math::rotmat2quat(est_tf.inverse().rotation());
  
  if (argc > 3){
    YAML::Emitter out;
    out << YAML::BeginMap; {
      out << YAML::Key << "scene";
      out << YAML::Value << sceneFile;

      out << YAML::Key << "config";
      out << YAML::Value << config;

      out << YAML::Key << "solutions";
      out << YAML::BeginSeq; {
        out << YAML::BeginMap; {
          out << YAML::Key << "models";
          out << YAML::Value << YAML::BeginSeq; {
              out << YAML::BeginMap; {
                out << YAML::Key << "model";
                out << YAML::Value << modelFile;
                out << YAML::Key << "q";
                out << YAML::Value << YAML::Flow << vector<double>(q_out.data(), q_out.data() + q_out.rows());
              } out << YAML::EndMap;
          } out << YAML::EndSeq;

          out << YAML::Key << "history";
          out << YAML::Value << YAML::BeginMap; {
            out << YAML::Key << "wall_time";
            out << YAML::Value << YAML::Flow << time;
          } out << YAML::EndMap;

        } out << YAML::EndMap;
      } out << YAML::EndSeq;
    } out << YAML::EndMap;

    ofstream fout(outputFile);
    fout << out.c_str();
    fout.close();
  }

  return 0;
}
