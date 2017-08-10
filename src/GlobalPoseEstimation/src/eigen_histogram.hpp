#pragma once

#include <Eigen/Dense>
#include <vector>

template <typename Scalar> class EigenHistogram {
public:
  EigenHistogram(
      const Eigen::Ref<const Eigen::VectorXi> n_bins,
      const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> lower_bounds,
      const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> upper_bounds);
  void Reset();
  void AddData(const Eigen::Matrix<Scalar, -1, -1> &data);

  void Deserialize(const std::vector<int> &histogram_data) {
    histogram_data_ = histogram_data;
  };
  const std::vector<int> &Serialize() { return histogram_data_; };

  // Exposed for testing
  int convert_full_index_to_linear_index(Eigen::VectorXi full_index);
  Eigen::VectorXi convert_linear_index_to_full_index(int linear_index);

private:
  Eigen::VectorXi n_bins_;
  Eigen::Matrix<Scalar, -1, 1> lower_bounds_;
  Eigen::Matrix<Scalar, -1, 1> upper_bounds_;
  std::vector<int> histogram_data_;
};