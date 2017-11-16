#pragma once

#include "Eigen/Dense"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointType>
static inline Eigen::Vector3d Vector3dFromPclPoint(PointType pc) {
  return Eigen::Vector3d(pc.x, pc.y, pc.z);
}

template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr convertMatrix3XdToPclPointXyzEtc(
    const Eigen::Matrix3Xd& input_points) {
  typename pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);
  for (int i = 0; i < input_points.cols(); i++) {
    PointType temp;
    temp.x = input_points(0, i);
    temp.y = input_points(1, i);
    temp.z = input_points(2, i);
    input_cloud->push_back(temp);
  }
  return input_cloud;
}

template <typename PointType>
Eigen::Matrix3Xd convertPclPointXyzEtcToMatrix3Xd(
    const typename pcl::PointCloud<PointType>::Ptr input_cloud) {
  Eigen::Matrix3Xd output_points(3, input_cloud->size());
  for (int i = 0; i < input_cloud->size(); i++) {
    const PointType& pt = input_cloud->points[i];
    output_points(0, i) = pt.x;
    output_points(1, i) = pt.y;
    output_points(2, i) = pt.z;
  }
  return output_points;
}

template <typename PointType>
Eigen::Matrix<double, 6, -1> convertPclPointXyzNormalEtcToMatrix6Xd(
    const typename pcl::PointCloud<PointType>::Ptr input_cloud) {
  Eigen::Matrix<double, 6, -1> output_points(6, input_cloud->size());
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

template <typename PointType>
std::vector<std::vector<double>> convertPclPointRgbEtcToVectorOfVectors(
    const typename pcl::PointCloud<PointType>::Ptr input_cloud) {
  std::vector<std::vector<double>> output_points;
  for (int i = 0; i < input_cloud->size(); i++) {
    const PointType& pt = input_cloud->points[i];
    output_points.push_back(std::vector<double>(
        {((double)pt.r) / 255., ((double)pt.g) / 255., ((double)pt.b) / 255.}));
  }
  return output_points;
}
