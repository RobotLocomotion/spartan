#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

Eigen::Matrix<double, 4, -1> SamplePointPairFeatures(
    const Eigen::Matrix<double, 6, -1> points_and_normals, const int n_features,
    double max_distance = std::numeric_limits<double>::infinity());