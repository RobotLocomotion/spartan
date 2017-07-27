#include "eigen_histogram.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;

template <typename Scalar>
EigenHistogram<Scalar>::EigenHistogram(
    const Eigen::Ref<const Eigen::VectorXi> n_bins,
    const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> lower_bounds,
    const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> upper_bounds)
    : n_bins_(n_bins), lower_bounds_(lower_bounds),
      upper_bounds_(upper_bounds) {
  assert(n_bins_.size() == lower_bounds_.size());
  assert(n_bins_.size() == upper_bounds_.size());
  assert(n_bins_.prod() > 0);
  histogram_data_.resize(n_bins_.prod());
  Reset();
}

template <typename Scalar> void EigenHistogram<Scalar>::Reset() {
  std::fill(histogram_data_.begin(), histogram_data_.end(), 0);
}

template <typename Scalar>
void EigenHistogram<Scalar>::AddData(
    const Eigen::Matrix<Scalar, -1, -1> &data) {
  assert(data.cols() > 0 && data.rows() == n_bins_.size());

  VectorXi full_index(n_bins_.size());
  for (int i = 0; i < data.cols(); i++) {
    full_index.setZero();

    // Calculate index into histogram
    for (int dim = 0; dim < n_bins_.size(); dim++) {
      double bin_fraction = (data(dim, i) - lower_bounds_(dim)) /
                            (upper_bounds_(dim) - lower_bounds_(dim));
      full_index(dim) =
          max(min((int)(bin_fraction * n_bins_(dim)), n_bins_(dim) - 1), 0);
    }
    histogram_data_[convert_full_index_to_linear_index(full_index)]++;
  }
}

template <typename Scalar>
int EigenHistogram<Scalar>::convert_full_index_to_linear_index(
    Eigen::VectorXi full_index) {
  assert(full_index.size() == n_bins_.size());

  int linear_index = 0;
  for (int i = 0; i < n_bins_.size(); i++) {
    assert(full_index[i] < n_bins_[i]);
    if (i == 0) {
      linear_index = full_index[i];
    } else {
      linear_index = linear_index * n_bins_[i] + full_index[i];
    }
  }
  return linear_index;
}

template <typename Scalar>
Eigen::VectorXi
EigenHistogram<Scalar>::convert_linear_index_to_full_index(int linear_index) {
  assert(linear_index < n_bins_.prod());
  VectorXi full_index(n_bins_.size());
  for (int i = n_bins_.size() - 1; i >= 0; i--) {
    full_index[i] = linear_index % n_bins_[i];
    linear_index /= n_bins_[i];
  }
  return full_index;
}

// Explicitly instantiates on the most common scalar types.
template class EigenHistogram<double>;