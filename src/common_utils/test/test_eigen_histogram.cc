#include <iostream>
#include "../eigen_histogram.h"

using namespace std;
using namespace Eigen;

int main() {
  VectorXi bins_many_different_dims(5);
  for (int i = 0; i < 5; i++) {
    bins_many_different_dims[i] = (rand() % 10) + 10;
  }
  VectorXd lb_many(5);
  VectorXd ub_many(5);
  EigenHistogram<double> test_histogram_big(bins_many_different_dims, lb_many,
                                            ub_many);
  for (int i = 0; i < bins_many_different_dims.prod(); i++) {
    if (i !=
        test_histogram_big.convert_full_index_to_linear_index(
            test_histogram_big.convert_linear_index_to_full_index(i))) {
      printf("Mismatch!\n");
    }
  }

  Matrix<double, 1, 6> test_matrix_1d;
  test_matrix_1d << 1.0, 2.0, 2.0, 3.0, 3.0, 3.0;

  VectorXi bins_1d = VectorXi::Ones(1) * 3;
  VectorXd lb_1d = VectorXd::Ones(1) * 0.5;
  VectorXd ub_1d = VectorXd::Ones(1) * 3.5;

  EigenHistogram<double> test_histogram_1d(bins_1d, lb_1d, ub_1d);
  test_histogram_1d.AddData(test_matrix_1d);

  cout << "1d: ";
  for (auto const &c : test_histogram_1d.Serialize()) cout << c << ' ';
  cout << endl;

  Matrix<double, 2, 4> test_matrix_2d;
  test_matrix_2d.col(0) = Vector2d(0.0, 0.0);
  test_matrix_2d.col(1) = Vector2d(0.0, 1.0);
  test_matrix_2d.col(2) = Vector2d(0.0, 1.0);
  test_matrix_2d.col(3) = Vector2d(1.0, 1.0);

  VectorXi bins_2d = VectorXi::Ones(2) * 2;
  VectorXd lb_2d = VectorXd::Ones(2) * 0.0;
  VectorXd ub_2d = VectorXd::Ones(2) * 1.0;

  EigenHistogram<double> test_histogram_2d(bins_2d, lb_2d, ub_2d);
  test_histogram_2d.AddData(test_matrix_2d);

  cout << "2d: ";
  for (auto const &c : test_histogram_2d.Serialize()) cout << c << ' ';
  cout << endl;
}