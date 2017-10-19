#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "common_utils/math_utils.h"
#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"

#include "drake/common/text_logging_gflags.h"
#include "drake/math/quaternion.h"

#include <gflags/gflags.h>
#include "spdlog/spdlog.h"

#include "RemoteTreeViewerWrapper.hpp"

#include "yaml-cpp/yaml.h"

DEFINE_string(config_filename, "config_filename",
              "YAML config file supplying search parameters.");

DEFINE_string(mesh_filename, "mesh",
              "VTP-openable mesh with colored vertices.");

DEFINE_string(labelfusion_data_dir, "data_dir",
              "Root of a LabelFusion log dir with a posegraph.posegraph and an "
              "images folder.");
// Assumption about labelfusion data:
//  the ith row (1-indexed) of the posegraph file corresponds to the rgb file
/// "%010d_rgb.png" % i

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Isometry3d;

using std::vector;
using std::string;

vector<Isometry3d> GeneratePoseList(string posegraph_filename) {
  auto console = spdlog::get("console");

  vector<Isometry3d> tfs;

  // Used in the inner loop repeatedly, so allocate once.
  double line_of_floats[8];
  ifstream file(posegraph_filename);
  string line;
  if (file.is_open()) {
    // Every line is a timestamp and then a 7-element pose
    while (getline(file, line)) {
      std::stringstream ss(line);
      for (int i = 0; i < 7; i++) {
        ss >> line_of_floats[i];
      }

      // Stick the 7-element pose into an isometry (xyz then quat)
      // and discard the timestamp
      Isometry3d tf;
      tf.matrix()(0, 3) = line_of_floats[1];
      tf.matrix()(1, 3) = line_of_floats[2];
      tf.matrix()(2, 3) = line_of_floats[3];
      tf.matrix().block<3, 3>(0, 0) = drake::math::quat2rotmat(
          Eigen::Map<const Vector4d>(line_of_floats + 4));
      tfs.push_back(tf);
    }
  } else {
    console->error("Couldn't open posegraph {0:s}", posegraph_filename);
    exit(1);
  }

  return tfs;
}

static int DoMain(void) {
  RemoteTreeViewerWrapper rm;
  auto console = spdlog::get("console");

  // Generate pose list from file
  string posegraph_filename =
      FLAGS_labelfusion_data_dir + "/posegraph.posegraph";
  auto tfs = GeneratePoseList(posegraph_filename);
  console->info("Loaded {0:d} poses from {1:s}", tfs.size(), posegraph_filename);

  string images_directory = FLAGS_labelfusion_data_dir + "/images/";

  // Open mesh with VTK
  vtkSmartPointer<vtkPolyData> meshPolyData =
      ReadPolyData(FLAGS_mesh_filename.c_str());
  console->info("Loaded {0:d} points from {1:s}", meshPolyData->GetNumberOfPoints(),
                FLAGS_mesh_filename);

  return 0;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  auto console = spdlog::stdout_color_mt("console");
  return DoMain();
}