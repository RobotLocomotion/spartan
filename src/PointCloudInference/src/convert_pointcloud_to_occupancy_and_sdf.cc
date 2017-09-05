#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "argagg.hpp"
#include "common/common.hpp"
#include "common/common_vtk.hpp"
#include "common/common_pcl_vtk.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>


#include "RemoteTreeViewerWrapper.hpp"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include "yaml-cpp/yaml.h"


using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointType;

int main(int argc, char** argv) {
  srand(0);

  argagg::parser argparser{
      {{"help", {"-h", "--help"}, "shows this help message", 0},
       {"pointcloud",
        {"-p", "--pointcloud"},
        "Point cloud file <openable by VTK>.",
        1},
       {"output",
        {"-o", "--output"},
        "Output file.",
        1}}};

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
  auto scenePCL = PointCloudFromPolyData(sceneVTK);

  pcl::PointCloud<pcl::PointXYZ>::Ptr gridPCL(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> grid_estimator;
  grid_estimator.initializeVoxelGrid();

  grid_estimator.setInputCloud (scenePCL);
  grid_estimator.setLeafSize (0.01f, 0.01f, 0.01f);
  grid_estimator.filter (*gridPCL);

  // Visualize the scene points and GT, to start with.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(convertPCLPointXYZToMatrix3Xd(scenePCL), {"pc_inference", "scenePCL"}, {{0.1, 1.0, 0.1}});
  rm.publishPointCloud(convertPCLPointXYZToMatrix3Xd(gridPCL), {"pc_inference", "gridPCL"}, {{0.1, 0.1, 1.0}});

  return 0;
}
