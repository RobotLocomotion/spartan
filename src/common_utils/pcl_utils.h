#pragma once

#include "Eigen/Dense"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointType>
static inline Eigen::Vector3d Vector3dFromPclPoint(PointType pc) {
  return Eigen::Vector3d(pc.x, pc.y, pc.z);
}