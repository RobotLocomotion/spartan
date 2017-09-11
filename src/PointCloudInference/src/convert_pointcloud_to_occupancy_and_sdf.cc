#include <unistd.h>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "argagg.hpp"
#include "common/common.hpp"
#include "common/common_pcl_vtk.hpp"
#include "common/common_vtk.hpp"
#include "eigen_nd_array.h"
#include "voxel_distance_field.h"

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "RemoteTreeViewerWrapper.hpp"

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkColorTransferFunction.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkImageShiftScale.h>
#include <vtkInteractorStyleFlight.h>
#include <vtkPiecewiseFunction.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSampleFunction.h>
#include <vtkSmartPointer.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkSphere.h>
#include <vtkVersion.h>
#include <vtkVolumeProperty.h>
#include <vtkXMLImageDataReader.h>
#include <vtksys/SystemTools.hxx>

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  srand(0);

  argagg::parser argparser{{
      {"help", {"-h", "--help"}, "shows this help message", 0},
      {"pointcloud",
       {"-p", "--pointcloud"},
       "Point cloud file <openable by VTK>.",
       1},
      {"output", {"-o", "--output"}, "Output file.", 1},
      {"voxel_size", {"-v", "--voxelsize"}, "Voxel size <in meters>.", 1},
      {"lower_bound_z",
       {"-l", "--lowerboundz"},
       "Lower bound (z) (optional)",
       1},
      {"upper_bound_z",
       {"-u", "--upperboundz"},
       "Upper bound (z) (optional)",
       1},
  }};

  argagg::parser_results args;
  try {
    args = argparser.parse(argc, argv);
  } catch (const exception& e) {
    cerr << e.what() << endl;
    return -1;
  }

  if (args["help"] || !args["pointcloud"] || !args["output"] ||
      !args["voxel_size"]) {
    cerr << argparser;
    return 0;
  }

  string sceneFile = args["pointcloud"].as<string>();
  string outputFile = args["output"].as<string>();

  auto sceneVTK = ReadPolyData(sceneFile.c_str());
  if (!sceneVTK){
    printf("Error loading %s\n", sceneFile.c_str());
    exit(-1);
  }
  auto scenePCL = PointCloudFromPolyDataWithRGB(sceneVTK);
  Matrix3Xd scenePts =
      convertPCLPointXYZToMatrix3Xd<pcl::PointXYZRGBA>(scenePCL);

  // Get bounding box of the whole point cloud
  Vector3d lb = scenePts.rowwise().minCoeff();
  Vector3d ub = scenePts.colwise().maxCoeff();

  if (args["lower_bound_z"]) lb[2] = args["lower_bound_z"].as<double>();
  if (args["upper_bound_z"]) ub[2] = args["upper_bound_z"].as<double>();

  Vector3i size =
      ((ub - lb).array() / args["voxel_size"].as<double>()).cast<int>();

  printf("Initializing VDF with dims %d, %d, %d...\n", size[0], size[1],
         size[2]);
  printf("\t and bounds [%f, %f, %f] <= x <= [%f, %f, %f]\n", lb[0], lb[1],
         lb[2], ub[0], ub[1], ub[2]);
  VoxelDistanceField vdf(size, lb, ub);

  double now = getUnixTime();
  printf("Adding %d points...\n", (int)scenePts.cols());
  vdf.AddPoints(scenePts);
  printf("Added %d points took %f seconds.\n", (int)scenePCL->points.size(),
         getUnixTime() - now);

  now = getUnixTime();
  vdf.UpdateOccupancy(1);
  printf("Updating occupancy took %f seconds.\n", getUnixTime() - now);

  vdf.Save(outputFile);
  printf("Saved to %s.\n", outputFile.c_str());

  return 0;
}
