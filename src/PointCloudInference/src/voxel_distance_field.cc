#include <zlib.h>
#include <algorithm>
#include <fstream>

#include "common/common.hpp"
#include "voxel_distance_field.h"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace Eigen;

VoxelDistanceField::VoxelDistanceField(
    const Eigen::Ref<const Eigen::Vector3i> size,
    const Eigen::Ref<const Eigen::Vector3d> lb,
    const Eigen::Ref<const Eigen::Vector3d> ub)
    : size_(size),
      counts_(size),
      distances_(size),
      known_(size),
      lb_(lb),
      ub_(ub),
      min_points_per_bin_(0),
      leaf_size_((ub_ - lb_).array() / size_.cast<double>().array()) {
  Reset();
}

VoxelDistanceField::VoxelDistanceField(const std::string& filename) {
  YAML::Node config = YAML::LoadFile(filename);
  assert(config["min_points_per_bin"]);
  min_points_per_bin_ = config["min_points_per_bin"].as<int>();
  assert(config["size"]);
  assert(config["lb"]);
  assert(config["ub"]);
  for (int i = 0; i < 3; i++) {
    size_[i] = config["size"].as<vector<int>>()[i];
    lb_[i] = config["lb"].as<vector<double>>()[i];
    ub_[i] = config["ub"].as<vector<double>>()[i];
  }
  counts_.Resize(size_);
  distances_.Resize(size_);
  known_.Resize(size_);
  leaf_size_ = ((ub_ - lb_).array() / size_.cast<double>().array());

  assert(config["counts"]);
  assert(config["distances"]);
  assert(config["known"]);

  auto counts_binary = config["counts"].as<YAML::Binary>();
  counts_.Deserialize(
      unzipStdVector<unsigned int>(counts_binary.data(), counts_binary.size()));
  auto distances_binary = config["distances"].as<YAML::Binary>();
  distances_.Deserialize(
      unzipStdVector<double>(distances_binary.data(), distances_binary.size()));
  auto known_binary = config["known"].as<YAML::Binary>();
  known_.Deserialize(
      unzipStdVector<char>(known_binary.data(), known_binary.size()));

  // Rebuild occupied nodes.
  occupied_nodes_.clear();
  for (int i = 0; i < size_[0]; i++) {
    for (int j = 0; j < size_[1]; j++) {
      for (int k = 0; k < size_[2]; k++) {
        Eigen::Vector3i bin_index({i, j, k});
        if (counts_.GetValue(bin_index) >= min_points_per_bin_) {
          occupied_nodes_.push_back(bin_index);
        }
      }
    }
  }
}

// Save self to a YAML file.
void VoxelDistanceField::Save(const std::string& filename) {
  YAML::Emitter out;

  out << YAML::BeginMap;
  {
    out << YAML::Key << "min_points_per_bin";
    out << YAML::Value << min_points_per_bin_;

    out << YAML::Key << "size";
    out << YAML::Value << EigenVectorToStdVector(size_);

    out << YAML::Key << "lb";
    out << YAML::Value << EigenVectorToStdVector(lb_);
    out << YAML::Key << "ub";
    out << YAML::Value << EigenVectorToStdVector(ub_);

    out << YAML::Key << "counts";
    auto counts_serialized = zipStdVector(counts_.Serialize());
    out << YAML::Value
        << YAML::Binary(counts_serialized.data(), counts_serialized.size());

    out << YAML::Key << "distances";
    auto distances_serialized = zipStdVector(distances_.Serialize());
    out << YAML::Value << YAML::Binary(distances_serialized.data(),
                                       distances_serialized.size());

    out << YAML::Key << "known";
    auto known_serialized = zipStdVector(known_.Serialize());
    out << YAML::Value
        << YAML::Binary(known_serialized.data(), known_serialized.size());
  }
  out << YAML::EndMap;

  ofstream fout(filename);
  fout << out.c_str();
  fout.close();
}

void VoxelDistanceField::Reset() {
  counts_.SetValue(0);
  distances_.SetValue(std::numeric_limits<double>::infinity());
  known_.SetValue(0);
  occupied_nodes_.clear();
}

Eigen::Vector3i VoxelDistanceField::ComputeBinIndexFromPoint(
    const Eigen::Ref<const Eigen::Vector3d>& point) {
  Eigen::Vector3i bin_index;
  for (int dim = 0; dim < 3; dim++) {
    double bin_fraction = (point(dim) - lb_(dim)) / (ub_(dim) - lb_(dim));
    // TODO(gizatt): More thoughtful handling of max-range. Right now, they get
    // lumped in the outer ring of bins.
    bin_index(dim) =
        max(min((int)(bin_fraction * size_(dim)), size_(dim) - 1), 0);
  }
  return bin_index;
}

Eigen::Vector3d VoxelDistanceField::ComputePointFromBinIndex(
    const Eigen::Ref<const Eigen::Vector3i>& bin_index) {
  return ((bin_index.cast<double>().array() / size_.cast<double>().array()) *
          (ub_ - lb_).array()) +
         lb_.array();
}

void VoxelDistanceField::AddPoints(
    const Eigen::Ref<const Eigen::Matrix3Xd> points) {
  for (int i = 0; i < points.cols(); i++) {
    auto point = points.col(i);
    Eigen::Vector3i bin_index = ComputeBinIndexFromPoint(point);
    // printf("Point (%f, %f, %f) -> (%d, %d, %d)\n", point[0], point[1],
    // point[2],
    //       bin_index[0], bin_index[1], bin_index[2]);
    // Yuck, gotta improve accessors on EigenNdArray...
    counts_.SetValue(counts_.GetValue(bin_index) + 1, bin_index);
  }
}

void VoxelDistanceField::DoRayTraversal(Eigen::Ref<Eigen::Vector3i> bin_index) {
  Vector3d original_point = ComputePointFromBinIndex(bin_index);
  Vector3d point = original_point;
  Vector3d direction = -point.normalized();

  known_.SetValue(1, bin_index);
  distances_.SetValue(0.0, bin_index);

  if (direction.norm() != direction.norm()) {
    printf("NAN norm: not doing traversal\n");
    return;
  }

  // This function references
  // http://www.cse.yorku.ca/~amana/research/grid.pdf for the stepping
  // logic and PCL's `voxel_grid_occlusion_estimation.hpp`
  // for a starting implementation.

  // The sign of `step_*` indicates the sign of the direction we
  // are traveling along that axis.
  int step_x, step_y, step_z;

  // The ray we follow is `point` + t * `direction`,
  // for normalized direction.
  // t_max_* indicates how far we must travel in the given
  // direction to leave the current voxel. min(t_max_*)
  // is thus the value of `t` at which we leave the
  // current voxel.
  // t_delta_* indicates how far we must travel in the given
  // direction to completely traverse a voxel (border-to-border).

  // Get us out of the starting voxel: figure out which way to
  // step to get us aligned onto the voxel borders.
  Vector3d voxel_corner = ComputePointFromBinIndex(bin_index);

  if (direction[0] >= 0) {
    voxel_corner[0] += leaf_size_[0];
    step_x = 1;
  } else {
    step_x = -1;
  }
  if (direction[1] >= 0) {
    voxel_corner[1] += leaf_size_[1];
    step_y = 1;
  } else {
    step_y = -1;
  }
  if (direction[2] >= 0) {
    voxel_corner[2] += leaf_size_[2];
    step_z = 1;
  } else {
    step_z = -1;
  }

  Vector3d t_max, t_delta;

  for (int i = 0; i < 3; i++) {
    if (direction[i] != 0.0) {
      t_max[i] = (voxel_corner[i] - point[i]) / direction[i];
      t_delta[i] = leaf_size_[i] / fabs(direction[i]);
    } else {
      t_max[i] = std::numeric_limits<double>::infinity();
      t_delta[i] = 0.0;
    }
  }

  // estimate next voxel (move off from initial voxel)
  if (t_max[0] <= t_max[1] && t_max[0] <= t_max[2]) {
    t_max[0] += t_delta[0];
    bin_index[0] += step_x;
  } else if (t_max[1] <= t_max[2] && t_max[1] <= t_max[0]) {
    t_max[1] += t_delta[1];
    bin_index[1] += step_y;
  } else {
    t_max[2] += t_delta[2];
    bin_index[2] += step_z;
  }

  // Using this I-thought-it-should-be-a-clear-inductive-assumption
  // doesn't work, and only labels *some* cells as known... I think
  // because it prematurely terminates (some view rays may intersect
  // and then diverge again due to state in t_max).
  while (1) {  // known_.GetValue(bin_index) == 0) {
    point = ComputePointFromBinIndex(bin_index);

    known_.SetValue(1, bin_index);
    double new_distance = (point - original_point).norm();
    if (new_distance < distances_.GetValue(bin_index)) {
      distances_.SetValue(new_distance, bin_index);
    }

    if (t_max[0] <= t_max[1] && t_max[0] <= t_max[2]) {
      t_max[0] += t_delta[0];
      bin_index[0] += step_x;
    } else if (t_max[1] <= t_max[2] && t_max[1] <= t_max[0]) {
      t_max[1] += t_delta[1];
      bin_index[1] += step_y;
    } else {
      t_max[2] += t_delta[2];
      bin_index[2] += step_z;
    }

    if ((bin_index.array() >= size_.array()).any() ||
        (bin_index.array() < Vector3i::Zero().array()).any()) {
      return;
    }
  }
}

void VoxelDistanceField::UpdateOccupancy(unsigned int min_points_per_bin) {
  min_points_per_bin_ = min_points_per_bin;
  for (int i = 0; i < size_[0]; i++) {
    for (int j = 0; j < size_[1]; j++) {
      for (int k = 0; k < size_[2]; k++) {
        Eigen::Vector3i bin_index({i, j, k});
        if (counts_.GetValue(bin_index) >= min_points_per_bin) {
          occupied_nodes_.push_back(bin_index);
          // Propagate known state, and TSDF (approximate SDF,
          // ignoring lateral directions that could be used) state,
          // backwards along view ray.
          DoRayTraversal(bin_index);
        }
      }
    }
  }
}

void VoxelDistanceField::RefineDistanceField() {
  // For every unprocessed node, starting with the known-occupied ones
  // in the queue and marked with themselves as their closest node:
  //   - Add all nodes closer to the camera (that are unmarked)
  //

  // ... i.e. Dijkstra's algorithm on a mesh grid.

  // Set those initial conditions properly:
  // Start with occupied nodes...

  /*
  std::vector<Eigen::Vector3i> process_queue = occupied_nodes;
  EigenNdArray<Eigen::Vector3i> closest_node(size_);
  closest_node.SetValue(Eigen::Vector3i({-1, -1, -1}));
  for (const auto& occupied_node : process_queue) {
    closest_node.SetValue(0.0, occupied_node);
  }

  for (int i = 0; i < size_[0]; i++) {
    for (int j = 0; j < size_[1]; j++) {
      for (int k = 0; k < size_[2]; k++) {
        Eigen::Vector3i bin_index = ComputeBinIndexFromPoint(point);
        if (counts_.GetValue(bin_index) >= min_points_per_bin) {
          distances_.SetValue(0, bin_index);
          known_.SetValue(1, bin_index);
          occupied_nodes.push_back(bin_index);
        }
      }
    }
  }
  */
  ;
}