#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "common_utils/math_utils.h"
#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"

#include <vtkImageCast.h>
#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/math/quaternion.h"

#include <gflags/gflags.h>
#include "spdlog/spdlog.h"

#include "RemoteTreeViewerWrapper.hpp"

#include "yaml-cpp/yaml.h"

/**
 * Given a mesh with good normals (in world frame),
 * and a sequence of RGB images with good intrinsics
 * and extrinsics in the frame of that mesh, generate
 * a list of color observations made of that vertex from
 * different perspectives. From this information, one could
 * conceivably jointly estimate lighting and material properties.
 *
 * Specifically:
 *   A vertex v transforms into a given camera frame by transformation:
 *     [u, v, 1]^T  = K TF [v_x v_y v_z 1]^T
 *   i.e. a standard pinhole camera model. TF is the camera pose, and
 *   K is an intrinsics matrix mapping from 3D pose in camera frame onto
 *   the image plane.
 *
 * First we populate the list of camera TFs across time, and store it separately
 * (as a TF is relevant for all pixels in an image).
 *
 * For every vertex, we construct a vertex_data_points list the stores a list
 * of observation events, each containing:
 *   - Index into the camera extrinsic TF list (1 byte)
 *   - RGB observation (3 bytes)
 *
 * Example approximate data usage computation:
 *    ~250000 vertex mesh
 *    ~4000 relevant different views into environment
 *     3 channels (RGB)
 *     = 3GB total data we'll feed into an optimization minimum.
 *
 *
 *    Of course, this can be dramatically reduced by cropping the input mesh.
 *
 *
 *  General TODOs:
 *    * Can probably replace this pipeline with vtkProjectedTexture and
 *      doing operations in texture space rather than on a per-vertex
 *      level. That'll be for a next pass, though.
 */

DEFINE_string(config_filename, "config_filename",
              "YAML config file supplying search parameters.");

DEFINE_string(mesh_filename, "mesh",
              "VTP-openable mesh with vertices with normals.");

DEFINE_string(
    labelfusion_data_dir, "data_dir",
    "Root of a LabelFusion log dir with a posegraph.posegraph file, "
    "a info.yaml file with camera intrinsics, "
    "and an images folder.");
// Assumption about labelfusion data:
//  the ith row (1-indexed) of the posegraph file corresponds to the rgb file
/// "%010d_rgb.png" % i

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using Eigen::Matrix3d;
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
      for (int i = 0; i < 8; i++) {
        ss >> line_of_floats[i];
      }

      // Stick the 7-element pose into an isometry (xyz then quat)
      // and discard the timestamp
      Isometry3d tf;
      tf.setIdentity();
      tf.matrix()(0, 3) = line_of_floats[1];
      tf.matrix()(1, 3) = line_of_floats[2];
      tf.matrix()(2, 3) = line_of_floats[3];
      // Posegraph.posegraph lists quat in xyzw order
      Vector4d quat(line_of_floats[7], line_of_floats[4], line_of_floats[5],
                    line_of_floats[6]);
      tf.matrix().block<3, 3>(0, 0) = drake::math::quat2rotmat(quat);

      tfs.push_back(tf.inverse());
    }
  } else {
    console->error("Couldn't open posegraph {0:s}", posegraph_filename);
    exit(1);
  }

  return tfs;
}

/**
 * Generates a camera instrinsic matrix from a config file
 * that supplies the center X and Y of the image plane,
 * and the focal length.
 * Generates K =
 *    [ f  0  u_x]
 *    [ 0  f  u_y]
 *    [ 0  0  1]
 * i.e. assumes 0 skew, and assumes focal length is supplied
 * in pixel units already.
 */
typedef Eigen::Matrix<double, 3, 3> CameraMatrixd;
CameraMatrixd GenerateCameraIntrinsics(string config_filename) {
  auto console = spdlog::get("console");
  CameraMatrixd K;
  K.setZero();

  YAML::Node config = YAML::LoadFile(config_filename);
  if (config["intrinsics"]) {
    double c_x = config["intrinsics"]["principalX"].as<double>();
    double c_y = config["intrinsics"]["principalY"].as<double>();

    K(0, 2) = c_x;
    K(1, 2) = c_y;

    double f = config["intrinsics"]["focalLength"].as<double>();
    K(0, 0) = f;
    K(1, 1) = -f; // Why is this flip necessary? I don't understand...
    K(2, 2) = 1.;

  } else {
    console->error("Config file had no intrinsics!");
    exit(1);
  }

  return K;
}

struct ColorObservation {
  int tf_index;
  unsigned char r;
  unsigned char g;
  unsigned char b;
};
struct VertexDataSet {
  Vector3d v;
  Vector3d n;
  std::vector<ColorObservation> color_observations;
};

void InitializeVertexDataSets(const vtkSmartPointer<vtkPolyData> meshPolyData,
                              const double bounding_box[6],
                              std::vector<VertexDataSet>* vertexDataSets,
                              std::map<int, int>* vertIndToDatasetInd) {
  auto console = spdlog::get("console");

  vertexDataSets->resize(meshPolyData->GetNumberOfPoints());
  vertIndToDatasetInd->clear();

  vtkDataArray* normalsGeneric = meshPolyData->GetPointData()->GetNormals();

  // Populate with some subset of vertices
  int k_good_vertices = 0;
  for (int i = 0; i < meshPolyData->GetNumberOfPoints(); i++) {
    auto vtk_pt = meshPolyData->GetPoint(i);

    if (vtk_pt[0] > bounding_box[0] && vtk_pt[0] < bounding_box[1] &&
        vtk_pt[1] > bounding_box[2] && vtk_pt[1] < bounding_box[3] &&
        vtk_pt[2] > bounding_box[4] && vtk_pt[2] < bounding_box[5]) {
      // This vertex should be included, so initialize its entry:
      double normals[3];
      normalsGeneric->GetTuple(0, normals);
      for (int k = 0; k < 3; k++) {
        vertexDataSets->at(k_good_vertices).v[k] = vtk_pt[k];
        //vertexDataSets->at(k_good_vertices).n[k] = normals[k];
      }
      vertexDataSets->at(k_good_vertices).color_observations.clear();
      vertIndToDatasetInd->insert(std::pair<int, int>(i, k_good_vertices));
      k_good_vertices++;
    }
  }

  vertexDataSets->resize(k_good_vertices);
}

static int DoMain(void) {
  RemoteTreeViewerWrapper rm;
  auto console = spdlog::get("console");

  // Generate pose list from file
  string posegraph_filename =
      FLAGS_labelfusion_data_dir + "/posegraph.posegraph";
  auto tfs = GeneratePoseList(posegraph_filename);
  console->info("Loaded {0:d} poses from {1:s}", tfs.size(),
                posegraph_filename);

  // Extract camera intrinsics matrix
  string config_filename = FLAGS_labelfusion_data_dir + "/info.yaml";
  CameraMatrixd K = GenerateCameraIntrinsics(config_filename);
  console->info("Loaded camera intrinsics:");
  std::cout << K << std::endl;

  // Open mesh with VTK
  vtkSmartPointer<vtkPolyData> meshPolyData =
      ReadPolyData(FLAGS_mesh_filename.c_str());
  console->info("Loaded {0:d} points from {1:s}",
                meshPolyData->GetNumberOfPoints(), FLAGS_mesh_filename);

  // Initialize the list of vertex sample info
  // Hack a bounding box for now, this should be a config parameter.
  double bb[6] = {-0.3, 0.3, -0.2, 0.2, 0.8, 1.1};
  vector<VertexDataSet> vertexDataSets;
  std::map<int, int> vertIndToDatasetInd;
  InitializeVertexDataSets(meshPolyData, bb, &vertexDataSets,
                           &vertIndToDatasetInd);
  console->info("Initialized {0:d} vertex data sets", vertexDataSets.size());

  // Iterate over images...
  for (int i = 1; i <= tfs.size(); i += 1000) {  // tfs.size(); i++){
    console->info("TF {0:d}:", i);
    Isometry3d tf = tfs[i - 1];
    std::cout << tf.matrix() << std::endl;

    char im_filename[1000];
    sprintf(im_filename, "%s/images/%010d_rgb.png",
            FLAGS_labelfusion_data_dir.c_str(), i);
    console->info("Loading picture {0:s}", im_filename);

    vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
    reader->SetFileName(im_filename);
    reader->Update();
    vtkSmartPointer<vtkImageData> imageData = reader->GetOutput();

    int* dims = imageData->GetDimensions();
    console->info("Loaded with dims {0:d}, {1:d}", dims[0], dims[1]);

    // For every vertex in the data set, project into the image
    int k = 0;

    for (auto& vertexDataSet : vertexDataSets) {
      Vector3d uv = K * tf * vertexDataSet.v;

      // And then rescale so that the 3rd element is 1
      uv /= uv[2];

      if (uv[0] >= 0 && uv[0] < dims[0] && uv[1] >= 0 && uv[1] < dims[1]) {
        unsigned char* pixel = static_cast<unsigned char*>(
            imageData->GetScalarPointer(uv[0], uv[1], 0));
        pixel[0] = 255;
        pixel[1] = 255;
        pixel[2] = 255;
        k++;
      }
    }

    char out_filename[1000];
    sprintf(out_filename, "%s/%010d_rgb_mask.png",
            FLAGS_labelfusion_data_dir.c_str(), i);
    console->info("Writing picture {0:s}, having touched {1:d} verts",
                  out_filename, k);

    vtkSmartPointer<vtkImageCast> castFilter =
        vtkSmartPointer<vtkImageCast>::New();
    castFilter->SetOutputScalarTypeToUnsignedChar();
    castFilter->SetInputData(imageData);
    castFilter->Update();

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(out_filename);
    writer->SetInputConnection(castFilter->GetOutputPort());
    writer->Write();
  }

  return 0;
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  auto console = spdlog::stdout_color_mt("console");
  return DoMain();
}