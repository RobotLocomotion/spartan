#include <iostream>
#include <cmath>
#include "voxel_distance_field.h"

#include "RemoteTreeViewerWrapper.hpp"

using namespace std;
using namespace Eigen;

int main() {
  RemoteTreeViewerWrapper rm;

  Vector3i size({5, 5, 5});
  Vector3d lb({0.0, 0.0, 0.0});
  Vector3d ub({5.0, 5.0, 5.0});
  VoxelDistanceField vdf(size, lb, ub);
  printf("Initialized...\n");

  Matrix3Xd points(3, 3);
  points.col(0) = Vector3d({2.0, 2.0, 2.0});
  points.col(1) = Vector3d({2.0, 5.0, 2.0});
  points.col(2) = Vector3d({4.0, 4.0, 4.0});

  rm.publishPointCloud(points, {"vdf_test", "points"}, {{1.0, 1.0, 1.0}});

  vdf.AddPoints(points);
  printf("Added points.\n");

  auto counts = vdf.GetCounts();
  Matrix3Xd counts_vis(3, size[0] * size[1] * size[2]);
  vector<vector<double>> counts_rgb(size[0] * size[1] * size[2]);
  int kk = 0;
  for (int i = 0; i < size[0]; i++) {
    for (int j = 0; j < size[1]; j++) {
      for (int k = 0; k < size[2]; k++) {
        Vector3i pt({i, j, k});
        counts_vis.col(kk) = vdf.ComputePointFromBinIndex(pt);
        int count = counts.GetValue(pt);
        counts_rgb[kk] =
            vector<double>({(double)min(count, 1), 0, 0});
        kk++;
      }
    } 
  }
  rm.publishPointCloud(counts_vis, {"vdf_test", "counts"}, counts_rgb);

  vdf.UpdateOccupancy(1);
  printf("Updated occupancy.\n");


  auto distances = vdf.GetDistances();
  auto known = vdf.GetKnown();

  // Visualize known
  Matrix3Xd known_vis(3, size[0] * size[1] * size[2]);
  vector<vector<double>> known_rgb(size[0] * size[1] * size[2]);
  kk = 0;
  for (int i = 0; i < size[0]; i++) {
    for (int j = 0; j < size[1]; j++) {
      for (int k = 0; k < size[2]; k++) {
        Vector3i pt({i, j, k});
        known_vis.col(kk) = vdf.ComputePointFromBinIndex(pt);
        int isknown = known.GetValue(pt);
        known_rgb[kk] =
            vector<double>({(double)min(isknown, 1), 0, 0});
        kk++;
      }
    } 
  }
  rm.publishPointCloud(known_vis, {"vdf_test", "known"}, known_rgb);

  // Visualize distances
  Matrix3Xd pre_distances_vis(3, size[0] * size[1] * size[2]);
  vector<vector<double>> pre_distances_rgb(size[0] * size[1] * size[2]);
  kk = 0;
  for (int i = 0; i < size[0]; i++) {
    for (int j = 0; j < size[1]; j++) {
      for (int k = 0; k < size[2]; k++) {
        Vector3i pt({i, j, k});
        pre_distances_vis.col(kk) = vdf.ComputePointFromBinIndex(pt);
        double distance = distances.GetValue(pt);
        printf("distance: %f\n", distance);
        if (distance > 1E20){
          pre_distances_rgb[kk] =
            vector<double>({0.0, 0.0, 0.0});
        } else {
          pre_distances_rgb[kk] =
            vector<double>({1. - distance/size[0], 0.0, 1. - distance/size[2]});
          }
        kk++;
      }
    } 
  }
  rm.publishPointCloud(pre_distances_vis, {"vdf_test", "pre_distances"}, pre_distances_rgb);

}