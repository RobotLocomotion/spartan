#pragma once

#include <Eigen/Dense>
#include <vector>

template <typename Scalar>
class EigenNdArray {
 public:
  EigenNdArray(const Eigen::Ref<const Eigen::VectorXi> dims);
  void Reset();

  void Deserialize(const std::vector<Scalar> &data) { data_ = data; };
  const std::vector<Scalar> &Serialize() const { return data_; };

  // Exposed for testing
  int convert_full_index_to_linear_index(Eigen::VectorXi full_index);
  Eigen::VectorXi convert_linear_index_to_full_index(int linear_index);

 protected:
  Eigen::VectorXi dims_;
  std::vector<Scalar> data_;
};