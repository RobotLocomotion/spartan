//
// Created by manuelli on 7/23/18.
//

#ifndef SPARTAN_UTILS_H
#define SPARTAN_UTILS_H

namespace spartan{
namespace drake_robot_control{
namespace utils{

// copied from http://docs.ros.org/jade/api/tf2_eigen/html/tf2__eigen_8h_source.html
inline
Eigen::Affine3d transformToEigen(const geometry_msgs::TransformStamped& t){
  return Eigen::Affine3d(Eigen::Translation3d(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z) * Eigen::Quaterniond(t.transform.rotation.w,t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z));
}

} // utils
} // drake_robot_control
} // spartan

#endif //SPARTAN_UTILS_H
