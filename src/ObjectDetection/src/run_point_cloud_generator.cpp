/*
 */

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"

#include "point_cloud_generator.hpp"
#include "RemoteTreeViewerWrapper.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc != 2){
    printf("Use: run_point_cloud_generator <config file>\n");
    exit(-1);
  }

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Cloud Generator Interface" << asctime(curtime);
  cout << "Config file " << string(argv[1]) << endl;
  cout << "***************************" << endl << endl;

  // Bring in config file
  string yamlString = string(argv[1]);
  YAML::Node config = YAML::LoadFile(yamlString);

  // Set up a point cloud generator
  if (config["point_cloud_generator_options"] == NULL){
    runtime_error("Config needs a point cloud generator option set.");
  }
  PointCloudGenerator pcg(config["point_cloud_generator_options"]);

  // Generate a point set
  Matrix3Xd scene_pts = pcg.samplePointCloud();

  // Visualize the results using the drake visualizer.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts, {"scene_pts"});
  rm.publishRigidBodyTree(pcg.get_robot(), pcg.get_q_robot(), Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});

  return 0;
}
