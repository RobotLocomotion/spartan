#pragma once

#include <Eigen/Dense>
#include <vector>

template <typename Scalar>
class EigenNdArray {
 public:
  EigenNdArray(const Eigen::Ref<const Eigen::VectorXi> n_bins);
  void Reset();

  void Deserialize(const std::vector<Scalar> &data) { data_ = data; };
  const std::vector<Scalar> &Serialize() const { return data_; };

  // Exposed for testing
  int convert_full_index_to_linear_index(Eigen::VectorXi full_index);
  Eigen::VectorXi convert_linear_index_to_full_index(int linear_index);

 protected:
  Eigen::VectorXi n_bins_;
  std::vector<Scalar> data_;
};

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