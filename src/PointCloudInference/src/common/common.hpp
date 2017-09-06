#pragma once

#include <Eigen/Dense>

template <typename Scalar, int N>
static inline typename std::vector<Scalar> EigenVectorToStdVector(
    const typename Eigen::Matrix<Scalar, N, 1>& vec) {
  return std::vector<Scalar>(vec.data(), vec.data() + vec.size());
}
template <typename Type>
static std::vector<unsigned char> zipStdVector(
    const typename std::vector<Type>& vec) {
  // TODO: actually use ZLIB.
  return std::vector<unsigned char>(
      (unsigned char*)const_cast<std::vector<Type>&>(vec).data(),
      (unsigned char*)(const_cast<std::vector<Type>&>(vec).data() +
                       vec.size()));
}
template <typename Type>
static std::vector<Type> unzipStdVector(
    const typename std::vector<unsigned char>& vec) {
  // TODO: actually use ZLIB.
  return std::vector<Type>(
      (Type*)const_cast<std::vector<unsigned char>&>(vec).data(),
      (Type*)(const_cast<std::vector<unsigned char>&>(vec).data() +
              vec.size()));
}
template <typename Type>
static std::vector<Type> unzipStdVector(const unsigned char* vec, int size) {
  // TODO: actually use ZLIB.
  return std::vector<Type>((const Type*)vec, (const Type*)(vec + size));
}

static double getUnixTime(void) {
  struct timespec tv;

  if (clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

  return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

// from https://forum.kde.org/viewtopic.php?f=74&t=91514
template <typename Derived>
static inline bool is_finite(const Eigen::MatrixBase<Derived>& x) {
  return ((x - x).array() == (x - x).array()).all();
}
template <typename Derived>
static inline bool is_nan(const Eigen::MatrixBase<Derived>& x) {
  return ((x.array() == x.array())).all();
}

static inline double randrange(double min, double max) {
  return (((double)rand()) / RAND_MAX) * (max - min) + min;
}

template <typename Scalar>
Scalar clamp(Scalar a, Scalar amin, Scalar amax) {
  return fmin(fmax(a, amin), amax);
}

static Eigen::Transform<double, 3, Eigen::Isometry> getAverageTransform(
    std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> transforms) {
  Eigen::Transform<double, 3, Eigen::Isometry> avg_transform;
  avg_transform.setIdentity();
  Eigen::Vector3d avg_pt = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_rots = Eigen::Matrix3d::Zero();
  int k = 0;
  for (auto iter = transforms.begin(); iter != transforms.end(); iter++) {
    avg_pt += iter->matrix().block<3, 1>(0, 3);
    sum_rots += iter->matrix().block<3, 3>(0, 0);
    k++;
  }

  if (k > 0) {
    avg_pt /= (double)k;
    // following tip from "A Note on Averaging Rotations", Curtis, Janin, Zikan,
    // 1993...
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        sum_rots, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d avg_rot = svd.matrixU() * svd.matrixV().transpose();
    avg_transform.matrix().block<3, 3>(0, 0) = avg_rot;
    avg_transform.matrix().block<3, 1>(0, 3) = avg_pt;
  }
  return avg_transform;
}

static Eigen::MatrixXd calculateHODDescriptors(const Eigen::Matrix3Xd& pts,
                                               int n_bins, double max_dist) {
  Eigen::MatrixXd hod_data(n_bins, pts.cols());
  hod_data.setZero();
  for (int i = 0; i < pts.cols(); i++) {
    for (int j = 0; j < pts.cols(); j++) {
      if (i != j) {
        double dist = (pts.col(j) - pts.col(i)).norm();
        if (dist <= max_dist) {
          int bin = clamp((int)(dist / max_dist * n_bins), 0, n_bins - 1);
          hod_data(bin, i) += 1.0;
        }
      }
    }
  }
  auto hod_colwise_sums = hod_data.colwise().sum();
  for (int j = 0; j < hod_data.cols(); j++) {
    for (int i = 0; i < n_bins; i++) {
      if (hod_colwise_sums(j) > 0) hod_data(i, j) /= hod_colwise_sums(j);
    }
  }
  return hod_data;
}

static double calculateAngleBetweenUnitVectors(
    const Eigen::Ref<Eigen::Vector3d> x, const Eigen::Ref<Eigen::Vector3d> y) {
  return atan2((x.cross(y)).norm(), x.dot(y));
}

// Adapted from https://www.gamedev.net/topic/552906-closest-point-on-triangle/
// and converted to Eigen by gizatt@mit.edu
static Eigen::Vector3d closestPointOnTriangle(
    const std::vector<Eigen::Vector3d>& triangle,
    const Eigen::Vector3d& sourcePosition) {
  Eigen::Vector3d edge0 = triangle[1] - triangle[0];
  Eigen::Vector3d edge1 = triangle[2] - triangle[0];
  Eigen::Vector3d v0 = triangle[0] - sourcePosition;

  float a = edge0.transpose() * edge0;
  float b = edge0.transpose() * edge1;
  float c = edge1.transpose() * edge1;
  float d = edge0.transpose() * v0;
  float e = edge1.transpose() * v0;

  float det = a * c - b * b;
  float s = b * e - c * d;
  float t = b * d - a * e;

  if (s + t < det) {
    if (s < 0.f) {
      if (t < 0.f) {
        if (d < 0.f) {
          s = clamp(-d / a, 0.f, 1.f);
          t = 0.f;
        } else {
          s = 0.f;
          t = clamp(-e / c, 0.f, 1.f);
        }
      } else {
        s = 0.f;
        t = clamp(-e / c, 0.f, 1.f);
      }
    } else if (t < 0.f) {
      s = clamp(-d / a, 0.f, 1.f);
      t = 0.f;
    } else {
      float invDet = 1.f / det;
      s *= invDet;
      t *= invDet;
    }
  } else {
    if (s < 0.f) {
      float tmp0 = b + d;
      float tmp1 = c + e;
      if (tmp1 > tmp0) {
        float numer = tmp1 - tmp0;
        float denom = a - 2 * b + c;
        s = clamp(numer / denom, 0.f, 1.f);
        t = 1 - s;
      } else {
        t = clamp(-e / c, 0.f, 1.f);
        s = 0.f;
      }
    } else if (t < 0.f) {
      if (a + d > b + e) {
        float numer = c + e - b - d;
        float denom = a - 2 * b + c;
        s = clamp(numer / denom, 0.f, 1.f);
        t = 1 - s;
      } else {
        s = clamp(-e / c, 0.f, 1.f);
        t = 0.f;
      }
    } else {
      float numer = c + e - b - d;
      float denom = a - 2 * b + c;
      s = clamp(numer / denom, 0.f, 1.f);
      t = 1.f - s;
    }
  }

  return triangle[0] + s * edge0 + t * edge1;
}