//
// Created by manuelli on 7/23/18.
//

#ifndef SPARTAN_UTILS_H
#define SPARTAN_UTILS_H

#include <Eigen/Dense>

// ROS
#include <geometry_msgs/TransformStamped.h>


namespace spartan{
namespace drake_robot_control{
namespace utils{

// copied from http://docs.ros.org/jade/api/tf2_eigen/html/tf2__eigen_8h_source.html
inline
Eigen::Affine3d transformToEigen(const geometry_msgs::TransformStamped& t){
  return Eigen::Affine3d(Eigen::Translation3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z) * Eigen::Quaterniond(t.transform.rotation.w,t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z));
}
inline
Eigen::Matrix3d HatOperator(const Eigen::Vector3d & p){
  Eigen::Matrix3d p_hat;
  p_hat << 0, -p(2), p(1),
      p(2), 0, -p(0),
      -p(1), p(0), 0;
  return p_hat;
};

inline
Eigen::Matrix<double, 6, 6> AdjointSE3(const Eigen::Matrix3d & R, const Eigen::Vector3d & p){

  Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix3d p_hat = HatOperator(p);
  Ad.topLeftCorner(3,3) = R;
  Ad.bottomLeftCorner(3,3) = p_hat * R;
  Ad.bottomRightCorner(3,3) = R;
  return Ad;
};

inline
Eigen::Vector3d LogSO3(const Eigen::Matrix3d & R){
  Eigen::AngleAxisd angle_axis(R);
  return angle_axis.angle() * angle_axis.axis();
}

} // utils
} // drake_robot_control
} // spartan

#endif //SPARTAN_UTILS_H
