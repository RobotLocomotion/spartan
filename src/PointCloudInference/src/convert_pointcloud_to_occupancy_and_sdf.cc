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

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "RemoteTreeViewerWrapper.hpp"

#include <vtkActor.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace Eigen;

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

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gridPCL(
      new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::VoxelGridOcclusionEstimation<pcl::PointXYZRGBA> grid_estimator;

  grid_estimator.setInputCloud(scenePCL);
  grid_estimator.setLeafSize(0.01, 0.01, 0.01);
  grid_estimator.initializeVoxelGrid();
  grid_estimator.filter(*gridPCL);

  // Visualize the downsampled cloud.
  RemoteTreeViewerWrapper rm;
  rm.publishPointCloud(
      convertPCLPointXYZToMatrix3Xd<pcl::PointXYZRGBA>(gridPCL),
      {"pc_inference", "gridPCL"},
      convertPCLPointRGBToVectorOfVectors<pcl::PointXYZRGBA>(gridPCL));

  // Extract out the ND array of the voxel grid
  Vector3i grid_size = grid_estimator.getNrDivisions();
  EigenNdArray<unsigned char> occupancy_grid(grid_size);

  for (int i = 0; i < grid_size(0); i++) {
    for (int j = 0; j < grid_size(1); j++) {
      for (int k = 0; k < grid_size(2); k++) {
        Vector3i here({i, j, k});
        int out_state;
        int ret = grid_estimator.occlusionEstimation(out_state, here);
        auto centroid = grid_estimator.getCentroidCoordinate(here);
        if (ret || out_state)
          cout << centroid.transpose() << ": " << out_state << " ( " << ret
               << " )" << endl;
        occupancy_grid.SetValue(-out_state,
                                here);  // -1 occluded, 0 free, 1 occupied
      }
    }
  }

  return 0;
}
