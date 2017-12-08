/*
 */

#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "RemoteTreeViewerWrapper.hpp"

#include "yaml-cpp/yaml.h"

#include "iterative_closest_point.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc != 3) {
    printf(
        "Use: run_iterative_closest_point_estimator <urdf file> <vtp point "
        "cloud>\n");
    exit(-1);
  }

  time_t _tm = time(NULL);
  struct tm* curtime = localtime(&_tm);

  // Bring in config file
  string urdf = string(argv[1]);
  string vtp = string(argv[2]);

  // Model will be a RigidBodyTree.
  RigidBodyTree<double> robot;
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(urdf, &robot);

  // Load point cloud
  Matrix3Xd scene_pts = LoadAndDownsamplePolyData(vtp);

  // Generate random initial seed
  VectorXd q_robot = VectorXd::Random(robot.get_num_positions(), 1);

  cout << "Q robot: " << q_robot.transpose() << endl;

  // Visualize the results using the drake visualizer.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts, {"scene_pts_loaded"});
  rm.publishRigidBodyTree(robot, q_robot, Vector4d(1.0, 0.2, 0.0, 0.5),
                          {"robot_initial_pose"});

  double last_publish_time = getUnixTime();
  for (int i = 0; i < 5000; i++) {
    q_robot = compute_articulated_icp_update_for_points(
        robot, q_robot, scene_pts, 0.0, 1.0, true);
    if (getUnixTime() - last_publish_time > 0.01){
      rm.publishRigidBodyTree(robot, q_robot, Vector4d(0.6, 0.2, 1.0, 0.5),
                              {"robot_updated_pose"});
      last_publish_time = getUnixTime();
    }
  }
  rm.publishRigidBodyTree(robot, q_robot, Vector4d(0.0, 1.0, 0.0, 0.7),
                          {"robot_last_pose"});

  return 0;
}
