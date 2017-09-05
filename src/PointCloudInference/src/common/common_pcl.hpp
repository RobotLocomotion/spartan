#pragma once

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

#include "RemoteTreeViewerWrapper.hpp"

using namespace std;
using namespace Eigen;

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

static pcl::PointCloud<pcl::PointNormal>::Ptr generateNormalsFromPoints(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
        int num_neighbors = 10, 
        bool orient_normals = false, 
        double interpoint_scale = -1., 
        int root_node_index = 0,
        Vector3d root_node_direction = {0.0, 0.0, 1.0}) {
  // Get rough scale and a guess at the interpoint distance in the cloud
  pcl::PointXYZ proj_min; 
  pcl::PointXYZ proj_max; 
  pcl::getMinMax3D(*input_cloud, proj_min, proj_max);

  printf("Passed interpoint %f\n", interpoint_scale);
  if (interpoint_scale <= 0){
    interpoint_scale = (pcl::geometry::distance(proj_max, proj_min) / 
                            powf( (double) input_cloud->size(), 0.33)) * sqrtf(3);
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
  ne.setKSearch (num_neighbors);
  ne.compute (*cloud_normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 
  pcl::concatenateFields(*input_cloud, *cloud_normals, *input_cloud_with_normals); 

  if (orient_normals) {
    printf("Unifying normals with interpoint scale %f... ", interpoint_scale);

    // Make normal edge directions consistently face upwards by flipping those that face downwards
    /*
    for (int i = 0; i < input_cloud->size(); i++) {
      doNormalConsensusWithVector3d(input_cloud_with_normals->points[i], {0.0, 0.0, 1.0});
    }
    */

    // Form a tree of all points, with edges connecting points within a given distance.
    std::vector<Node> graph;

    for (int i = 0; i < input_cloud->size(); i++) {
      Node new_node;
      new_node.fixed = false;
      tree->radiusSearch (input_cloud->points[i], interpoint_scale, new_node.outgoing_edges_inds, new_node.outgoing_edges_weights);
      graph.push_back(new_node);
    }
    
    int next = root_node_index;
    auto this_norm = doNormalConsensusWithVector3d(input_cloud_with_normals->points[next], root_node_direction);
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
          next_queue.push_back({i, this_norm});
        }
      }
    }

    printf("done\n");
  }
  return input_cloud_with_normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertMatrix3XdToPCLPointXYZ(const Eigen::Matrix3Xd& input_points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < input_points.cols(); i++) {
    pcl::PointXYZ temp;
    temp.x = input_points(0, i);
    temp.y = input_points(1, i);
    temp.z = input_points(2, i);
    input_cloud->push_back(temp);
  }
  return input_cloud;
}

// TODO(gizatt): Can certainly be collapsed more with
// some more modularization of the point vs normal vs rgb
// handling. Or template magic.
template <typename PointT>
Matrix3Xd convertPCLPointXYZToMatrix3Xd(
    const typename pcl::PointCloud<PointT>::Ptr input_cloud) {
  Matrix3Xd output_points(3, input_cloud->size());
  for (int i = 0; i < input_cloud->size(); i++) {
    const PointT& pt = input_cloud->points[i];
    output_points(0, i) = pt.x;
    output_points(1, i) = pt.y;
    output_points(2, i) = pt.z;
  }
  return output_points;
}
template <typename PointT>
std::vector<std::vector<double>> convertPCLPointRGBToVectorOfVectors(
    const typename pcl::PointCloud<PointT>::Ptr input_cloud) {
  std::vector<std::vector<double>> output_points;
  for (int i = 0; i < input_cloud->size(); i++) {
    const PointT& pt = input_cloud->points[i];
    output_points.push_back(std::vector<double>(
        {((double)pt.r) / 255., ((double)pt.g) / 255., ((double)pt.b) / 255.}));
  }
  return output_points;
}
template <typename PointT>
Matrix<double, 6, -1> convertPCLPointNormalToMatrix6Xd(
    const typename pcl::PointCloud<PointT>::Ptr input_cloud) {
  Matrix<double, 6, -1> output_points(6, input_cloud->size());
  for (int i = 0; i < input_cloud->size(); i++) {
    const pcl::PointNormal& pt = input_cloud->points[i];
    output_points(0, i) = pt.x;
    output_points(1, i) = pt.y;
    output_points(2, i) = pt.z;
    output_points(3, i) = pt.normal_x;
    output_points(4, i) = pt.normal_y;
    output_points(5, i) = pt.normal_z;
  }
  return output_points;
}

static Eigen::Matrix3Xd generateNormalsFromMatrix3Xd(const Matrix3Xd& input_points, const std::vector<std::string>& prefix = {}, bool orient_normals = true, bool with_vis = true) {
  Vector3d bb_min = input_points.rowwise().minCoeff();
  Vector3d bb_max = input_points.rowwise().maxCoeff();
  double diag = (bb_max - bb_min).norm();
  double interpoint_scale = (diag / powf( (double) input_points.size(), 0.33)) * sqrtf(3);

  auto input_cloud = convertMatrix3XdToPCLPointXYZ(input_points);

  int highest_node_ind; input_points.row(2).maxCoeff(&highest_node_ind);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = generateNormalsFromPoints(input_cloud, 10, orient_normals, interpoint_scale, highest_node_ind, {0.0, 0.0, 1.0});

  std::vector<string> label;
  RemoteTreeViewerWrapper rm;
  if (with_vis) {
    label.insert(label.end(), prefix.begin(), prefix.end());
    label.push_back("pointsandfeatures");
    label.push_back("points");
    rm.publishPointCloud(input_points, label);
  }

  Points output_points;
  Feature output_features;
  for (int i = 0; i < input_points.cols(); i++) {
    const pcl::PointNormal &pt = cloud_normals->points[i];

    if (with_vis && i % (input_points.cols() / 100) == 0) {
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

  return convertPCLPointNormalToMatrix6Xd<pcl::PointNormal>(cloud_normals).block(3, 0, 3, input_points.cols());
}

std::pair<Points, Feature> generatePointsAndFPFHFeaturesFromPoints(const Eigen::Matrix3Xd& input_points, 
  const std::vector<std::string>& prefix = {}, bool orient_normals = false, bool with_vis = true, 
  double interpoint_scale = -1, double feature_radius_multiplier = 10.0){

  if (interpoint_scale <= 0.0) {
    Vector3d bb_min = input_points.rowwise().minCoeff();
    Vector3d bb_max = input_points.rowwise().maxCoeff();
    double diag = (bb_max - bb_min).norm();
    interpoint_scale = (diag / powf( (double) input_points.size(), 0.33)) * sqrtf(3);
  }
  auto input_cloud = convertMatrix3XdToPCLPointXYZ(input_points);

  int highest_node_ind; input_points.row(2).maxCoeff(&highest_node_ind);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals = generateNormalsFromPoints(input_cloud, 5, orient_normals, interpoint_scale, highest_node_ind, {0.0, 0.0, 1.0});

  // For all model and scene points, generate FPFH features
  // (this code is modified from https://github.com/IntelVCL/FastGlobalRegistration)
  pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>());
  fest.setRadiusSearch(interpoint_scale*feature_radius_multiplier);  
  fest.setInputCloud(cloud_normals);
  fest.setInputNormals(cloud_normals);
  fest.compute(*object_features);

  std::vector<string> label;
  RemoteTreeViewerWrapper rm;
  if (with_vis) {
    label.insert(label.end(), prefix.begin(), prefix.end());
    label.push_back("pointsandfeatures");
    label.push_back("points");
    rm.publishPointCloud(input_points, label);
  }

  Points output_points;
  Feature output_features;
  for (int i = 0; i < input_points.cols(); i++) {
    const pcl::PointNormal &pt = cloud_normals->points[i];
    output_points.push_back({pt.x, pt.y, pt.z});
    const pcl::FPFHSignature33 &feature = object_features->points[i];
    output_features.push_back(Map<const VectorXf>(feature.histogram, 33));

    if (with_vis && i % (input_points.cols() % 10) == 0) {
      Vector3d normal = Map<const Vector3f>(pt.normal).cast<double>();
      Matrix<double, 3, 2> normal_line;
      normal_line.col(0) = input_points.col(i);
      normal_line.col(1) = input_points.col(i) + normal*0.05;
      char buf[20]; sprintf(buf, "normal_%d", i);
      label.clear();  
      label.insert(label.end(), prefix.begin(), prefix.end());
      label.push_back("pointsandfeatures");
      label.push_back("normals");
      label.push_back(buf);
      rm.publishLine(normal_line, label);
    }
  }
  return {output_points, output_features};
}