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
#include "common/common_vtk.hpp"
#include "yaml-cpp/yaml.h"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include "RemoteTreeViewerWrapper.hpp"
#include "GoICP_V1.3/jly_goicp.h"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

int main(int argc, char** argv) {
  srand(0);

  if (argc < 3){
    printf("Use: run_goicp_detector <scene cloud, vtp> <model cloud, vtp> <config file> <optional output_file>\n");
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
  cout << "GoICP Object Pose Estimator" << asctime(curtime);
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
  auto goicp_config = config["detector_options"];

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
  rm.publishPointCloud(scene_pts, {"scene_pts_downsampled"}, {{0.1, 1.0, 1.0}});
  //rm.publishPointCloud(model_pts_in, {"model_pts"}, {{0.1, 1.0, 1.0}});
  rm.publishPointCloud(model_pts, {"model_pts_downsampled"}, {{0.1, 1.0, 1.0}});
  //rm.publishRigidBodyTree(robot, q_robot, Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});


  // Load in GoICP Estimator
  
  GoICP goicp;

  // Set config for goicp
  goicp.MSEThresh = goicp_config["MSEThresh"].as<float>();
  goicp.initNodeRot.a = goicp_config["rotMinX"].as<float>();
  goicp.initNodeRot.b = goicp_config["rotMinY"].as<float>();
  goicp.initNodeRot.c = goicp_config["rotMinZ"].as<float>();
  goicp.initNodeRot.w = goicp_config["rotWidth"].as<float>();
  goicp.initNodeTrans.x = goicp_config["transMinX"].as<float>();
  goicp.initNodeTrans.y = goicp_config["transMinY"].as<float>();
  goicp.initNodeTrans.z = goicp_config["transMinZ"].as<float>();
  goicp.initNodeTrans.w = goicp_config["transWidth"].as<float>();
  goicp.trimFraction = goicp_config["trimFraction"].as<float>();
  goicp.dt.SIZE = goicp_config["distTransSize"].as<float>();
  goicp.dt.expandFactor = goicp_config["distTransExpandFactor"].as<float>();

  // If < 0.1% trimming specified, do no trimming
  if(goicp.trimFraction < 0.001)
  {
    goicp.doTrim = false;
  }


  // Shift centroids to origin
  Vector3d scenePtAvg = scene_pts.rowwise().mean();
  Vector3d modelPtAvg = model_pts.rowwise().mean();
  scene_pts.colwise() -= scenePtAvg;
  model_pts.colwise() -= modelPtAvg;

  // Load model and data point clouds into GoICP
  double max_abs_coeff = fmax(
      fmax(fabs(model_pts.minCoeff()), fabs(model_pts.maxCoeff())),
      fmax(fabs(scene_pts.minCoeff()), fabs(scene_pts.maxCoeff()))
    );
  printf("Max abs coeff %f\n", max_abs_coeff);
  scene_pts /= (2.0*max_abs_coeff);
  model_pts /= (2.0*max_abs_coeff);

/*
  rm.publishPointCloud(scene_pts, {"scene_pts_downsampled_rescaled"}, {{0.0, 1.0, 0.0}});
  //rm.publishPointCloud(model_pts_in, {"model_pts"}, {{0.1, 1.0, 1.0}});
  rm.publishPointCloud(model_pts, {"model_pts_downsampled_rescaled"}, {{1.0, 0.0, 0.0}});
*/

  vector<POINT3D> pModel, pData;
  for (int i=0; i<scene_pts.cols(); i++){
    POINT3D new_pt;
    new_pt.x = scene_pts(0, i);
    new_pt.y = scene_pts(1, i);
    new_pt.z = scene_pts(2, i);
    pData.push_back(new_pt);
  }

/*
  // actually no
  Affine3d tf;
  tf.setIdentity();
  AngleAxisd rollAngle(0.1, Vector3d::UnitZ());
  AngleAxisd yawAngle(0.1, Vector3d::UnitY());
  AngleAxisd pitchAngle(0.1, Vector3d::UnitX());
  Quaterniond q = rollAngle*yawAngle*pitchAngle;
  tf.matrix().block<3, 3>(0, 0) = q.matrix();
  model_pts = tf * scene_pts;
  */

  for (int i=0; i<model_pts.cols(); i++) {
    POINT3D new_pt;
    new_pt.x = model_pts(0, i);
    new_pt.y = model_pts(1, i);
    new_pt.z = model_pts(2, i);
    pModel.push_back(new_pt);
  }

  // Flip scene and model so "data" is the smaller cloud
  // (to take advantage of the constant-time factor for
  // the model cloud)
  bool flipped = false;
  if (pModel.size() > pData.size()){
    goicp.pModel = pModel.data();
    goicp.Nm = pModel.size();
    goicp.pData = pData.data();
    goicp.Nd = pData.size();
  } else {
    goicp.pModel = pData.data();
    goicp.Nm = pData.size();
    goicp.pData = pModel.data();
    goicp.Nd = pModel.size();
    flipped = true;
  }

  cout << "Building Distance Transform, " << goicp.Nm << " vs " << goicp.Nd << "..." << flush;
  clock_t clockBegin = clock();
  goicp.BuildDT();
  clock_t clockEnd = clock();
  cout << (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC << "s (CPU)" << endl;

  cout << "Registering..." << endl;
  clockBegin = clock();
  goicp.Register();
  clockEnd = clock();
  double time = (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC;
  cout << "Optimal Rotation Matrix:" << endl;
  cout << goicp.optR << endl;
  cout << "Optimal Translation Vector:" << endl;
  cout << goicp.optT << endl;
  cout << "Finished in " << time << endl;

  // Un-scale the point clouds
  scene_pts *= 2.0*max_abs_coeff;
  model_pts *= 2.0*max_abs_coeff;

  // And un-center them
  scene_pts.colwise() += scenePtAvg;
  model_pts.colwise() += modelPtAvg;

  // Publish the transformed scene point cloud (I'm transforming the scene because
  // the model is usually better centered )
  Affine3d est_tf;
  est_tf.setIdentity();
  for (int i = 0; i < 3; i++) {
    est_tf.translation()[i] = goicp.optT.val[i][0]*2.0*max_abs_coeff;
    for (int j = 0; j < 3; j++) {
      est_tf.matrix()(i, j) = goicp.optR.val[i][j];
    }
  }
  if (flipped) {
    est_tf.translation() += (scenePtAvg - est_tf.rotation() * modelPtAvg);
    est_tf = est_tf.inverse();
  } else {
    est_tf.translation() += (modelPtAvg - est_tf.rotation() * scenePtAvg);
  }

  cout << "Est tf " << est_tf.matrix() << endl;
  cout << "Est tf inv" << est_tf.inverse().matrix() << endl;
  rm.publishPointCloud(est_tf * scene_pts, {"scene_pts_tf"}, {{0.5, 0.5, 1.0}});

  return 0;
}
