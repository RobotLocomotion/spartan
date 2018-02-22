#include "point_cloud_generator.hpp"

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "yaml-cpp/yaml.h"
#include "common/common.hpp"

#include "RemoteTreeViewerWrapper.hpp"

using namespace std;
using namespace Eigen;
using namespace drake::parsers::urdf;

PointCloudGenerator::PointCloudGenerator(const YAML::Node& config){
  config_ = config;

  // Load the model itself
  if (config_["models"] == NULL){
    runtime_error("Must specify models for point cloud generator to work with.");
  }
  // Model will be a RigidBodyTree.
  q_robot_.resize(0);
  int old_q_robot_size = 0;
  for (auto iter=config_["models"].begin(); iter!=config_["models"].end(); iter++){
    string urdf = (*iter)["urdf"].as<string>();
    AddModelInstanceFromUrdfFileWithRpyJointToWorld(urdf, &robot_);
    // And add initial state info that we were passed
    vector<double> q0 = (*iter)["q0"].as<vector<double>>();
    assert(robot_.get_num_positions() - old_q_robot_size == q0.size());
    q_robot_.conservativeResize(robot_.get_num_positions());
    for (int i=0; i<q0.size(); i++){
      q_robot_[old_q_robot_size] = q0[i];
      old_q_robot_size++; 
    }
  }
}

Eigen::Matrix3Xd PointCloudGenerator::samplePointCloud(){
  int sampling_mode = -1;
  if (config_["scene_sampling_mode"]){
    sampling_mode = config_["scene_sampling_mode"].as<int>();
  } else {
    runtime_error("Please set a scene_sampling_mode in the pcg options"
      "to use the samplePointCloud() method.");
  }

  if (sampling_mode >= 0 && sampling_mode < 10)
    return samplePointCloudFromSurface();
  else
    return samplePointCloudFromRaycast();
}

Eigen::Matrix3Xd PointCloudGenerator::samplePointCloudFromSurface(){
  // Retrieve relevant options from config
  int opt_num_rays = 100;
  if (config_["num_rays"])
    opt_num_rays = config_["num_rays"].as<int>();

  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_);

  Matrix3Xd all_vertices;
  DrakeShapes::TrianglesVector all_faces;
  vector<double> face_cumulative_area = {0.0};

  // Collect faces from all bodies in this configuration
  for (auto iter = robot_.get_bodies().begin(); iter != robot_.get_bodies().end(); iter++) {
    for (const auto& collision_elem : (*iter)->get_collision_element_ids()) {
      auto element = robot_.FindCollisionElement(collision_elem);
      if (element->hasGeometry()){
        const DrakeShapes::Geometry & geometry = element->getGeometry();
        if (geometry.hasFaces()){
          Matrix3Xd points;
          geometry.getPoints(points);
          // Transform them to this robot's frame
          points = element->getLocalTransform() * points;
          points = robot_.transformPoints(robot_kinematics_cache, points, (*iter)->get_body_index(), 0);

          all_vertices.conservativeResize(3, points.cols() + all_vertices.cols());
          all_vertices.block(0, all_vertices.cols() - points.cols(), 3, points.cols()) = points;
          DrakeShapes::TrianglesVector faces;
          geometry.getFaces(&faces);
          // Calculate the face surface area, so we can do even sampling from the surface.
          // See http://math.stackexchange.com/a/128999.
          // Also use this loop to offset the vertex indices to the appropriate all_vertices indices.
          for (auto& face : faces){
            Vector3d a = points.col( face[0] );
            Vector3d b = points.col( face[1] );
            Vector3d c = points.col( face[2] );
            double area = ((b - a).cross(c - a)).norm() / 2.;
            face_cumulative_area.push_back(face_cumulative_area[face_cumulative_area.size()-1] + area);

            face[0] += (all_vertices.cols() - points.cols());
            face[1] += (all_vertices.cols() - points.cols());
            face[2] += (all_vertices.cols() - points.cols());
          }
          all_faces.insert(all_faces.end(), faces.begin(), faces.end());
        } 
      }
    }
  }

  // Normalize cumulative areas.
  for (int i=0; i<face_cumulative_area.size(); i++){
    face_cumulative_area[i] /= face_cumulative_area[face_cumulative_area.size() - 1];
  }

  Eigen::Matrix3Xd pc(3, opt_num_rays);
  int i = 0;
  while (i < opt_num_rays){
    // Pick the face we'll sample from
    double sample = randrange(1E-12, 1.0 - 1E-12);
    int k = 0;
    for (k=0; k<face_cumulative_area.size(); k++){
      if (face_cumulative_area[k] >= sample){
        break;
      }
    }
    k -= 1;

    Vector3d a = all_vertices.col(all_faces[k][0]);
    Vector3d b = all_vertices.col(all_faces[k][1]);
    Vector3d c = all_vertices.col(all_faces[k][2]);

    double s1 = randrange(0.0, 1.0); 
    double s2 = randrange(0.0, 1.0);

    if (s1 + s2 <= 1.0){
      Vector3d pt = a + 
                    s1 * (b - a) +
                    s2 * (c - a);
      pc.col(i) = pt;
      i++;
    }
  }

  return pc;
}

Eigen::Matrix3Xd PointCloudGenerator::samplePointCloudFromRaycast(){
  // Retrieve relevant options from config
  int opt_x_dim = 320;
  double opt_x_fov = 90; // degrees
  int opt_y_dim = 240;
  double opt_y_fov = 90; // degrees
  double opt_max_range = 30.0;
  if (config_["x_dim"])
    opt_x_dim = config_["x_dim"].as<int>();
  if (config_["y_dim"])
    opt_y_dim = config_["y_dim"].as<int>();
  if (config_["x_fov"])
    opt_x_fov = config_["x_fov"].as<double>();
  if (config_["y_fov"])
    opt_y_fov = config_["y_fov"].as<double>();
  if (config_["max_range"])
    opt_max_range = config_["max_range"].as<double>();

  // For now, we are going to assume the camera origin is positioned at the
  // world origin, with +Z forward, +Y up, +X left.

  KinematicsCache<double> robot_kinematics_cache = robot_.doKinematics(q_robot_);


  Matrix3Xd origins = Matrix3Xd::Zero(3, opt_x_dim*opt_y_dim);
  Matrix3Xd endpoints = Matrix3Xd::Zero(3, opt_x_dim*opt_y_dim);

  for (int u=0; u<opt_x_dim; u++){
    for (int v=0; v<opt_y_dim; v++){
      double x_angle = ( ((double)u / (double)opt_x_dim)-0.5 )*opt_x_fov*(3.1415/180.0);
      double y_angle = ( ((double)v / (double)opt_y_dim)-0.5 )*opt_y_fov*(3.1415/180.0);
      // When both angles are zero, we should output max_range * [0, 0, 1]
      // When x increases positively, we should output max_range * [<0, 0, <1]
      // When y increases positively, we should output max_range * [0, <0, <1]
      // Not 100% sure in my math here yet but it'll get us close
      // TODO(gizatt) do real geometry
      endpoints.col(u*opt_y_dim + v) = Vector3d(
         -sin(x_angle),
         -sin(y_angle),
          cos(x_angle)*cos(y_angle)
        );
    }  
  }

  VectorXd distances;
  robot_.collisionRaycast(robot_kinematics_cache, origins, endpoints, distances, false);

  // Assemble the final point cloud
  Eigen::Matrix3Xd pc(3, opt_x_dim*opt_y_dim);
  for (int i=0; i<opt_x_dim*opt_y_dim; i++){
    Vector3d ray = (endpoints.col(i) - origins.col(i));
    ray /= ray.norm();
    if (distances(i) > 0){
      pc.col(i) = origins.col(i) + ray*distances(i);
    } else {
      pc.col(i) = origins.col(i) + ray*opt_max_range;
    }
  }

  return pc;
}