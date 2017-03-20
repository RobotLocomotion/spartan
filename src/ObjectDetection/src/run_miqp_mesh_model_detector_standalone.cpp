/*
 */

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/lcm/drake_lcm.h"

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"
#include "optimization_helpers.h"
#include "rotation_helpers.h"

#include "RemoteTreeViewerWrapper.hpp"

#include "point_cloud_generator.hpp"
#include "miqp_mesh_model_detector.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;
using namespace drake::systems;


int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc != 2){
    printf("Use: miqp_multiple_mesh_models_detector_drake <config file>\n");
    exit(-1);
  }

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "MIQP Multiple Mesh Models Detector, Drake Ver, " << asctime(curtime);
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
  Matrix3Xd scene_pts = pcg.samplePointCloudFromSurface();
  
  // Visualize the results using the drake visualizer.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts, {"scene_pts"});
  rm.publishRigidBodyTree(pcg.get_robot(), pcg.get_q_robot(), Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});

  // And set up our detector
  if (config["detector_options"] == NULL){
    runtime_error("Config needs a detector option set.");
  }
  MIQPMultipleMeshModelDetector detector(config["detector_options"]);
  auto solutions = detector.doObjectDetection(scene_pts);

  for (const auto& solution : solutions){
    printf("printing sol\n");
    stringstream sol_name;
    sol_name << "sol_obj_" << solution.objective;

    for (const auto& detection : solution.detections){
      const RigidBody<double>& body = detector.get_robot().get_body(detection.obj_ind);

      for (const auto& collision_elem_id : body.get_collision_element_ids()) {
        stringstream collision_elem_id_str;
        collision_elem_id_str << collision_elem_id;
        auto element = detector.get_robot().FindCollisionElement(collision_elem_id);
        if (element->hasGeometry()){
          vector<string> path;
          path.push_back(sol_name.str());
          path.push_back(body.get_name());
          path.push_back(collision_elem_id_str.str());
          rm.publishGeometry(element->getGeometry(), detection.est_tf * element->getLocalTransform(), Vector4d(0.3, 1.0, 0.3, 0.2), path);
        }
      }
    }
  }

  return 0;
}
