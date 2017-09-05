#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

#include "eigen_nd_array.h"

/**
 *
 * This class assembles a voxel-wise signed distance field
 * from a pointcloud, *assuming that the point cloud is
 * observed from the origin* to assign the sign and observability.
 *
 * To use, call:
 *   AddPointCloud() and then UpdateOccupancy() and then
 *   UpdateDistanceFieldAndObservability()
 *
 */
class VoxelDistanceField {
 public:
  VoxelDistanceField(const Eigen::Ref<const Eigen::Vector3i> size,
                     const Eigen::Ref<const Eigen::Vector3d> lb,
                     const Eigen::Ref<const Eigen::Vector3d> ub);

  void Reset();

  const EigenNdArray<unsigned int>& GetCounts() { return counts_; }
  const EigenNdArray<double>& GetDistances() { return distances_; }
  const EigenNdArray<char>& GetKnown() { return known_; }

  // Adds these points to the appropriate cells to keep track of
  // counts.
  void AddPoints(const Eigen::Ref<const Eigen::Matrix3Xd> points);

  void DoRayTraversal(Eigen::Ref<Eigen::Vector3i> bin_index);
  void UpdateOccupancy(unsigned int min_points_per_bin);

  /*
   * UpdateOccupancy creates an *approximate* distance field which
   * only considers distance forward and backward from the
   * observed points (from the camera perspective). This function
   * refines that distance field via Dijkstra's algorithm starting
   * from the occupied node layer. The approximate distance field
   * provides an initial guess that may save computation for surfaces
   * where a straight-back correspondence is the correct shortest
   * distance.
   */

  void RefineDistanceField();

  // Internal helpers
  Eigen::Vector3i ComputeBinIndexFromPoint(
      const Eigen::Ref<const Eigen::Vector3d>& point);
  Eigen::Vector3d ComputePointFromBinIndex(
    const Eigen::Ref<const Eigen::Vector3i>& bin_index);

  bool MoveToNextNodeInDirection(
      Eigen::Ref<Eigen::Vector3d> point, Eigen::Ref<Eigen::Vector3i> bin_index,
      const Eigen::Ref<const Eigen::Vector3d> direction);

 protected:
  // Number of points reported in this bin:
  EigenNdArray<unsigned int> counts_;
  // Distance from each bin to the known occupied bins,
  // positive towards camera, negative away from camera:
  EigenNdArray<double> distances_;
  EigenNdArray<char> known_;

  Eigen::Vector3i size_;
  std::vector<Eigen::Vector3i> occupied_nodes_;
  Eigen::Vector3d lb_;
  Eigen::Vector3d ub_;
  Eigen::Vector3d leaf_size_;
};