#include "eigen_nd_array.h"

using namespace std;
using namespace Eigen;

template <typename Scalar>
EigenNdArray<Scalar>::EigenNdArray(
    const Eigen::Ref<const Eigen::VectorXi> dims)
    : dims_(dims) {
  assert(dims_.prod() > 0);
  data_.resize(dims_.prod());
  Reset();
}

template <typename Scalar>
void EigenNdArray<Scalar>::Reset() {
  std::fill(data_.begin(), data_.end(), 0);
}

template <typename Scalar>
int EigenNdArray<Scalar>::convert_full_index_to_linear_index(
    Eigen::VectorXi full_index) {
  assert(full_index.size() == dims_.size());

  int linear_index = 0;
  for (int i = 0; i < dims_.size(); i++) {
    assert(full_index[i] < dims_[i]);
    if (i == 0) {
      linear_index = full_index[i];
    } else {
      linear_index = linear_index * dims_[i] + full_index[i];
    }
  }
  return linear_index;
}

template <typename Scalar>
Eigen::VectorXi EigenNdArray<Scalar>::convert_linear_index_to_full_index(
    int linear_index) {
  assert(linear_index < dims_.prod());
  VectorXi full_index(dims_.size());
  for (int i = dims_.size() - 1; i >= 0; i--) {
    full_index[i] = linear_index % dims_[i];
    linear_index /= dims_[i];
  }
  return full_index;
}

template class EigenNdArray<int>;
template class EigenNdArray<double>;