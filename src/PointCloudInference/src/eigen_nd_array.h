#pragma once

#include <Eigen/Dense>
#include <vector>

template <typename Scalar>
class EigenNdArray {
 public:
  // Default construction is to start empty.
  EigenNdArray();
  EigenNdArray(const Eigen::Ref<const Eigen::VectorXi> size);
  void Resize(const Eigen::Ref<const Eigen::VectorXi> size);
  void SetValue(Scalar value);
  void SetValue(Scalar value,
                const Eigen::Ref<const Eigen::VectorXi> full_index) {
    data_[convert_full_index_to_linear_index(full_index)] = value;
  }
  Scalar GetValue(const Eigen::Ref<const Eigen::VectorXi> full_index) {
    return data_[convert_full_index_to_linear_index(full_index)];
  }

  void Deserialize(const std::vector<Scalar> &data) { data_ = data; };
  const std::vector<Scalar> &Serialize() const { return data_; };

  // Exposed for testing
  int convert_full_index_to_linear_index(Eigen::VectorXi full_index);
  Eigen::VectorXi convert_linear_index_to_full_index(int linear_index);

 protected:
  Eigen::VectorXi size_;
  std::vector<Scalar> data_;
};