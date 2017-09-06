#include "eigen_nd_array.h"
#include <iostream>

using namespace std;
using namespace Eigen;

template <typename Scalar>
EigenNdArray<Scalar>::EigenNdArray() : size_(VectorXi::Zero(0)) {
  ;
}

template <typename Scalar>
EigenNdArray<Scalar>::EigenNdArray(const Eigen::Ref<const Eigen::VectorXi> size)
    : size_(size) {
  assert(size_.prod() > 0);
  data_.resize(size_.prod());
}

template <typename Scalar>
void EigenNdArray<Scalar>::Resize(const Eigen::Ref<const Eigen::VectorXi> size) {
  assert(size.prod() >= 0);
  size_ = size;
  data_.resize(size_.prod());
}

template <typename Scalar>
void EigenNdArray<Scalar>::SetValue(Scalar value) {
  std::fill(data_.begin(), data_.end(), value);
}

template <typename Scalar>
int EigenNdArray<Scalar>::convert_full_index_to_linear_index(
    Eigen::VectorXi full_index) {
  assert(full_index.size() == size_.size());

  int linear_index = 0;
  for (int i = 0; i < size_.size(); i++) {
    assert(full_index[i] < size_[i]);
    if (i == 0) {
      linear_index = full_index[i];
    } else {
      linear_index = linear_index * size_[i] + full_index[i];
    }
  }
  return linear_index;
}

template <typename Scalar>
Eigen::VectorXi EigenNdArray<Scalar>::convert_linear_index_to_full_index(
    int linear_index) {
  assert(linear_index < size_.prod());
  VectorXi full_index(size_.size());
  for (int i = size_.size() - 1; i >= 0; i--) {
    full_index[i] = linear_index % size_[i];
    linear_index /= size_[i];
  }
  return full_index;
}

template class EigenNdArray<int>;
template class EigenNdArray<unsigned int>;
template class EigenNdArray<Eigen::Vector3i>;
template class EigenNdArray<double>;
template class EigenNdArray<float>;
template class EigenNdArray<bool>;
template class EigenNdArray<char>;
template class EigenNdArray<unsigned char>;