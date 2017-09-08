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

const int kSizeX = 100;
const int kSizeY = 100;
const int kSizeZ = 100;
int main(int argc, char** argv) {
  srand(0);

  argagg::parser argparser{
      {{"help", {"-h", "--help"}, "shows this help message", 0},
       {"pointcloud",
        {"-p", "--pointcloud"},
        "Point cloud file <openable by VTK>.",
        1},
       {"output", {"-o", "--output"}, "Output file.", 1}}};

  argagg::parser_results args;
  try {
    args = argparser.parse(argc, argv);
  } catch (const exception& e) {
    cerr << e.what() << endl;
    return -1;
  }

  if (args["help"] || !args["pointcloud"] || !args["output"]) {
    cerr << argparser;
    return 0;
  }

  string sceneFile = args["pointcloud"].as<string>();
  string outputFile = args["output"].as<string>();

  auto sceneVTK = ReadPolyData(sceneFile.c_str());
  auto scenePCL = PointCloudFromPolyDataWithRGB(sceneVTK);

  Vector3i size({kSizeX, kSizeY, kSizeZ});

  Vector3d lb({-2., -2., 0.5});
  Vector3d ub({2., 2., 2.0});
  VoxelDistanceField vdf(size, lb, ub);
  printf("Initialized vdf...\n");

  double now = getUnixTime();
  vdf.AddPoints(convertPCLPointXYZToMatrix3Xd<pcl::PointXYZRGBA>(scenePCL));
  printf("Added %d points took %f seconds.\n", (int)scenePCL->points.size(),
         getUnixTime() - now);

  now = getUnixTime();
  vdf.UpdateOccupancy(1);
  printf("Updating occupancy took %f seconds.\n", getUnixTime() - now);

  vdf.Save(outputFile);
  printf("Saved to %s.\n", outputFile.c_str());

  return 0;
}
