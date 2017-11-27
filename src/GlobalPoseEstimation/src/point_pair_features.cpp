#include "point_pair_features.hpp"
#include "common_utils/math_utils.h"

using Eigen::Vector3d;

Eigen::Matrix<double, 4, -1>
SamplePointPairFeatures(const Eigen::Matrix<double, 6, -1>& points_and_normals,
                        const int n_features, double max_distance) {
  int n_pts = points_and_normals.cols();
  Eigen::Matrix<double, 4, -1> features(4, n_features);
  int i = 0;
  while (i < n_features) {
    int i_1 = rand() % n_pts;
    int i_2 = rand() % n_pts;
    if (i_1 == i_2)
      continue;

    Vector3d d = points_and_normals.block<3, 1>(0, i_1) -
                 points_and_normals.block<3, 1>(0, i_2);
    double distance = d.norm();
    if (distance < 1E-6 || distance > max_distance) {
      // Duplicate point... try again.
      continue;
    }
    d /= d.norm();
    Vector3d n_1 = points_and_normals.block<3, 1>(3, i_1);
    n_1 /= n_1.norm();
    Vector3d n_2 = points_and_normals.block<3, 1>(3, i_2);
    n_2 /= n_2.norm();

    features(0, i) = distance;
    features(1, i) = calculateAngleBetweenUnitVectors(n_1, n_2);
    features(2, i) = calculateAngleBetweenUnitVectors(d, n_1);
    features(3, i) = calculateAngleBetweenUnitVectors(d, n_2);
    i++;
  }
  return features;
}