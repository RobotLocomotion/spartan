#pragma once

#include "eigen_nd_array.h"

template <typename Scalar>
class EigenHistogram : public EigenNdArray<int> {
 public:
  EigenHistogram(
      const Eigen::Ref<const Eigen::VectorXi> n_bins,
      const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> lower_bounds,
      const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> upper_bounds);
  void AddData(const Eigen::Matrix<Scalar, -1, -1> &data);

 private:
  Eigen::Matrix<Scalar, -1, 1> lower_bounds_;
  Eigen::Matrix<Scalar, -1, 1> upper_bounds_;
};