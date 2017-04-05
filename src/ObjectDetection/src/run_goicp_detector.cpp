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
#include "yaml-cpp/yaml.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "RemoteTreeViewerWrapper.hpp"
#include "GoICP_V1.3/jly_goicp.h"
#include "point_cloud_generator.hpp" // using this for model surface sampling

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  srand(0);

  if (argc < 3){
    printf("Use: run_goicp_detector <point cloud file> <config file> <optional output_file>\n");
    exit(-1);
  }

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "GoICP Object Pose Estimator" << asctime(curtime);
  cout << "Point cloud file " << string(argv[1]) << endl;
  cout << "Config file " << string(argv[2]) << endl;
  if (argc > 3)
    cout << "Output file " << string(argv[3]) << endl;
  cout << "***************************" << endl << endl;

  // Bring in config file
  string pcdFile = string(argv[1]);
  string yamlString = string(argv[2]);
  YAML::Node config = YAML::LoadFile(yamlString);

  if (config["detector_options"] == NULL){
    runtime_error("Need detector options.");
  }

  // Set up model
  if (config["detector_options"]["models"] == NULL){
    runtime_error("Model must be specified.");
  }

  if (config["detector_options"] == NULL){
    runtime_error("Config needs a detector option set.");
  }
  auto goicp_config = config["detector_options"];

  // Model will be a RigidBodyTree which we'll sample a model point cloud from
  // for GoICP.
  RigidBodyTree<double> robot;
  VectorXd q_robot;
  int old_q_robot_size = 0;
  for (auto iter=config["models"].begin(); iter!=config["models"].end(); iter++){
    string urdf = (*iter)["urdf"].as<string>();
    AddModelInstanceFromUrdfFileWithRpyJointToWorld(urdf, &robot);
    // And add initial state info that we were passed
    vector<double> q0 = (*iter)["q0"].as<vector<double>>();
    assert(robot.get_num_positions() - old_q_robot_size == q0.size());
    q_robot.conservativeResize(robot.get_num_positions());
    for (int i=0; i<q0.size(); i++){
      q_robot[old_q_robot_size] = q0[i];
      old_q_robot_size++; 
    }
  }
  robot.compile();

  // Load point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile<PointType>( pcdFile, cloud );
  cout << "Loaded " << cloud.size() << " points from " << pcdFile << endl;
  Matrix3Xd scene_pts_in(3, cloud.size());
  for (int i=0; i<cloud.size(); i++){
    scene_pts_in(0, i) = cloud.at(i).x;
    scene_pts_in(1, i) = cloud.at(i).y;
    scene_pts_in(2, i) = cloud.at(i).z;
  }
  Matrix3Xd scene_pts;
  srand(0);
  if (!goicp_config["downsample_to_this_many_points"]) {
    scene_pts = scene_pts_in;
  } else {
    int optDownsampleToThisManyPoints = goicp_config["downsample_to_this_many_points"].as<int>();
    scene_pts.resize(3, optDownsampleToThisManyPoints);
    VectorXi indices = VectorXi::LinSpaced(scene_pts_in.cols(), 0, scene_pts_in.cols());
    // always seed this the same way
    srand(0);
    std::random_shuffle(indices.data(), indices.data() + scene_pts_in.cols());
    for (int i = 0; i < optDownsampleToThisManyPoints; i++) {
      scene_pts.col(i) = scene_pts_in.col(indices[i]);
    }    
  }


  // Visualize the scene points and GT, to start with.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts_in, {"scene_pts_loaded"}, {0.1, 1.0, 0.1});
  rm.publishPointCloud(scene_pts, {"scene_pts_downsampled"}, {0.1, 1.0, 1.0});
  rm.publishRigidBodyTree(robot, q_robot, Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});


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

  // Load model and data point clouds

  // First sample model point with point cloud generator
  if (config["sampler_options"] == NULL){
    runtime_error("Need options for the model sampling point cloud generator.");
  }
  PointCloudGenerator pcg(config["sampler_options"]);
  Matrix3Xd model_pts = pcg.samplePointCloudFromSurface();
  rm.publishPointCloud(model_pts, {"model_pts_sampled"}, {1.0, 0.1, 0.1});

  double max_abs_coeff = fmax(
      fmax(fabs(model_pts.minCoeff()), fabs(model_pts.maxCoeff())),
      fmax(fabs(scene_pts.minCoeff()), fabs(scene_pts.maxCoeff()))
    );
  printf("Max abs coeff %f\n", max_abs_coeff);
  scene_pts /= (2.0*max_abs_coeff);
  model_pts /= (2.0*max_abs_coeff);

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

  goicp.pModel = pModel.data();
  goicp.Nm = pModel.size();
  goicp.pData = pData.data();
  goicp.Nd = pData.size();

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
  return 0;
}
