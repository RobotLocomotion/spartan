#pragma once

#include "RemoteTreeViewerWrapper.hpp"
#include <string>
#include <Eigen/Dense>

static void publishErrorColorCodedPointCloud(Eigen::Matrix3Xd tf_scene_pts, Eigen::Matrix3Xd model_pts, std::string opt_name){
  std::vector<std::vector<double>> colors;

  Eigen::VectorXd dists(tf_scene_pts.cols());

  for (int i = 0; i < tf_scene_pts.cols(); i++){ 
    dists(i) = std::numeric_limits<double>::infinity();
    dists(i) = ((model_pts.colwise() - tf_scene_pts.col(i)).colwise().norm()).minCoeff();
  }

  double max_dist = 0.02; //dists.maxCoeff();

  for (int i = 0; i < tf_scene_pts.cols(); i++){ 
    colors.push_back({
      clamp(dists(i) / max_dist, 0.0, 1.0),
      clamp(1.0 - dists(i) / max_dist, 0.0, 1.0),
      clamp(1.0 - 2*fabs(dists(i) / max_dist - 0.5), 0.0,1.0)
    });
  }
  RemoteTreeViewerWrapper rm;
  rm.publishPointCloud(tf_scene_pts, {opt_name, "scene_pts_tf"}, colors);
}