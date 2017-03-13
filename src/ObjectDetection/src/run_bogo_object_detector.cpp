/*
 */

#include "common/common.hpp"

#include "drake/common/eigen_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "RemoteTreeViewerWrapper.hpp"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc < 2){
    printf("Use: run_bogo_object_detector <config file> <optional output_file>\n");
    exit(-1);
  }

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Bogo (Random) Object Detector" << asctime(curtime);
  cout << "Config file " << string(argv[1]) << endl;
  if (argc > 2)
    cout << "Output file " << string(argv[2]) << endl;
  cout << "***************************" << endl << endl;

  // Bring in config file
  string yamlString = string(argv[1]);
  YAML::Node config = YAML::LoadFile(yamlString);

  // Set up model
  if (config["models"] == NULL){
    runtime_error("Model must be specified.");
  }
  // Model will be a RigidBodyTree.
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

  // Load point cloud
  if (config["point_cloud"] == NULL){
    runtime_error("Point cloud must be specified.");
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile<PointType>( config["point_cloud"].as<string>(), cloud);
  cout << "Loaded " << cloud.size() << " points from " << config["point_cloud"].as<string>() << endl;

  Matrix3Xd scene_pts(3, cloud.size());
  for (int i=0; i<cloud.size(); i++){
    scene_pts(0, i) = cloud.at(i).x;
    scene_pts(1, i) = cloud.at(i).y;
    scene_pts(2, i) = cloud.at(i).z;
  }

  // Generate random pose estimate
  q_robot = VectorXd::Random(q_robot.rows(), 1);

  // Visualize the results using the drake visualizer.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts, {"scene_pts_loaded"});
  rm.publishRigidBodyTree(robot, q_robot, Vector4d(0.6, 1.0, 0.0, 0.2), {"robot_bogo_fitted"});

  if (argc > 2){
    string outputFilename = string(argv[2]);

    YAML::Emitter out;
    out << YAML::BeginSeq;
    
    out << YAML::BeginMap;
    out << YAML::Key << "pose";
    out << YAML::Value << YAML::Flow << vector<double>(q_robot.data(), q_robot.data() + q_robot.rows());
    out << YAML::Key << "score";
    out << YAML::Value << 0.0;
    out << YAML::EndMap;

    out << YAML::EndSeq;

    ofstream fout(outputFilename);
    fout << out.c_str();
    fout.close();
  }

  return 0;
}
