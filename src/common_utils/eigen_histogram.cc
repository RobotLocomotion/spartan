#include "eigen_histogram.h"
#include <iostream>

using namespace std;
using namespace Eigen;

template <typename Scalar>
EigenHistogram<Scalar>::EigenHistogram(
    const Eigen::Ref<const Eigen::VectorXi> dims,
    const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> lower_bounds,
    const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> upper_bounds)
    : EigenNdArray<int>(dims),
      lower_bounds_(lower_bounds),
      upper_bounds_(upper_bounds) {
  assert(dims_.size() == lower_bounds_.size());
  assert(dims_.size() == upper_bounds_.size());
  assert(dims_.prod() > 0);
  data_.resize(dims_.prod());
  Reset();
}

template <typename Scalar>
void EigenHistogram<Scalar>::AddData(
    const Eigen::Matrix<Scalar, -1, -1> &data) {
  assert(data.cols() > 0 && data.rows() == dims_.size());

  VectorXi full_index(dims_.size());
  for (int i = 0; i < data.cols(); i++) {
    full_index.setZero();

    // Calculate index into histogram
    for (int dim = 0; dim < dims_.size(); dim++) {
      double bin_fraction = (data(dim, i) - lower_bounds_(dim)) /
                            (upper_bounds_(dim) - lower_bounds_(dim));
      full_index(dim) =
          max(min((int)(bin_fraction * dims_(dim)), dims_(dim) - 1), 0);
    }
    data_[convert_full_index_to_linear_index(full_index)]++;
  }
}

// Explicitly instantiates on the most common scalar types.
template class EigenHistogram<double>;