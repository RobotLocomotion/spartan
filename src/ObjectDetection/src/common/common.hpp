#ifndef MANIP_TRACKING_COMMON_H
#define MANIP_TRACKING_COMMON_H

#include <Eigen/Dense>

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

// from https://forum.kde.org/viewtopic.php?f=74&t=91514
template<typename Derived>
static inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
   return ( (x - x).array() == (x - x).array()).all();
}
template<typename Derived>
static inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
   return ((x.array() == x.array())).all();
}

static inline double randrange(double min, double max){
  return (((double)rand()) / RAND_MAX)*(max - min) + min;
}

static Eigen::Transform<double, 3, Eigen::Isometry>
getAverageTransform(std::vector<Eigen::Transform<double, 3, Eigen::Isometry>> transforms){
  Eigen::Transform<double, 3, Eigen::Isometry> avg_transform;
  avg_transform.setIdentity();
  Eigen::Vector3d avg_pt = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_rots = Eigen::Matrix3d::Zero();
  int k=0;
  for (auto iter=transforms.begin(); iter!=transforms.end(); iter++){
    avg_pt += iter->matrix().block<3, 1>(0, 3);
    sum_rots += iter->matrix().block<3, 3>(0, 0);
    k++;
  }
  
  if (k > 0){
    avg_pt /= (double)k;
    // following tip from "A Note on Averaging Rotations", Curtis, Janin, Zikan, 1993...
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sum_rots,  Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d avg_rot = svd.matrixU() * svd.matrixV().transpose();
    avg_transform.matrix().block<3, 3>(0, 0) = avg_rot;
    avg_transform.matrix().block<3, 1>(0, 3) = avg_pt;
  }
  return avg_transform;
}

#endif