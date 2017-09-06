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

const int kSizeX = 200;
const int kSizeY = 200;
const int kSizeZ = 200;

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

  Vector3d lb({-2., -2., -2.});
  Vector3d ub({2., 2., -0.5});
  VoxelDistanceField vdf(size, lb, ub);
  printf("Initialized vdf...\n");

  double now = getUnixTime();
  vdf.AddPoints(convertPCLPointXYZToMatrix3Xd<pcl::PointXYZRGBA>(scenePCL));
  printf("Added %d points took %f seconds.\n", (int)scenePCL->points.size(),
         getUnixTime() - now);

  now = getUnixTime();
  vdf.UpdateOccupancy(10);
  printf("Updating occupancy took %f seconds.\n", getUnixTime() - now);

  for (const auto& point : scenePCL->points) {
    ;
  }

  // VOLUME RENDERING WITH VTK

  auto known = vdf.GetKnown();

  // Store the relevant volumes into vtkImageData
  vtkSmartPointer<vtkImageData> imageData_known =
      vtkSmartPointer<vtkImageData>::New();
  imageData_known->SetDimensions(kSizeX, kSizeY, kSizeZ);
#if VTK_MAJOR_VERSION <= 5
  imageData_known->SetNumberOfScalarComponents(1);
  imageData_known->SetScalarTypeToUnsignedChar();
#else
  imageData_known->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
#endif
  int* dims = imageData_known->GetDimensions();
  // Fill every entry of the image data with "2.0"
  for (int z = 0; z < dims[2]; z++) {
    for (int y = 0; y < dims[1]; y++) {
      for (int x = 0; x < dims[0]; x++) {
        unsigned char* pixel =
            static_cast<unsigned char*>(imageData_known->GetScalarPointer(x, y, z));
        pixel[0] = (unsigned char)known.GetValue(Vector3i({x, y, z}));
        if (x == y && y == z)
          printf("Pixel %d, %d, %d: %d\n", x, y, z, pixel[0]);
      }
    }
  }

  // Set up the render window.
  vtkSmartPointer<vtkRenderWindow> renWin =
      vtkSmartPointer<vtkRenderWindow>::New();
  vtkSmartPointer<vtkRenderer> ren1 = vtkSmartPointer<vtkRenderer>::New();
  ren1->SetBackground(0.0, 0.0, 0.0);

  renWin->AddRenderer(ren1);

  renWin->SetSize(1024, 768);

  vtkSmartPointer<vtkRenderWindowInteractor> iren =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleFlight> style =
      vtkSmartPointer<vtkInteractorStyleFlight>::New();
  iren->SetInteractorStyle(style);
  iren->SetRenderWindow(renWin);
  renWin->Render();  // make sure we have an OpenGL context.

  // Set up the volume rendering.
  vtkSmartPointer<vtkSmartVolumeMapper> volumeMapper =
      vtkSmartPointer<vtkSmartVolumeMapper>::New();
  volumeMapper->SetBlendModeToComposite();  // composite first
#if VTK_MAJOR_VERSION <= 5
  volumeMapper->SetInputConnection(imageData_known->GetProducerPort());
#else
  volumeMapper->SetInputData(imageData_known);
#endif
  vtkSmartPointer<vtkVolumeProperty> volumeProperty =
      vtkSmartPointer<vtkVolumeProperty>::New();
  volumeProperty->ShadeOff();
  volumeProperty->SetInterpolationType(VTK_LINEAR_INTERPOLATION);

  vtkSmartPointer<vtkPiecewiseFunction> compositeOpacity =
      vtkSmartPointer<vtkPiecewiseFunction>::New();
  compositeOpacity->AddPoint(0, 0.0);
  compositeOpacity->AddPoint(1, 1.0);
  compositeOpacity->AddPoint(1000, 1.0);
  volumeProperty->SetScalarOpacity(compositeOpacity);  // composite first.

  vtkSmartPointer<vtkColorTransferFunction> color =
      vtkSmartPointer<vtkColorTransferFunction>::New();
  color->AddRGBPoint(0, 0.0, 0.0, 1.0);
  color->AddRGBPoint(1, 1.0, 0.0, 0.0);
  color->AddRGBPoint(2, 1.0, 1.0, 1.0);
  color->AddRGBPoint(1000, 1.0, 1.0, 1.0);
  volumeProperty->SetColor(color);

  vtkSmartPointer<vtkVolume> volume = vtkSmartPointer<vtkVolume>::New();
  volume->SetMapper(volumeMapper);
  volume->SetProperty(volumeProperty);
  ren1->AddViewProp(volume);
  ren1->ResetCamera();

// 3D texture mode if possible
//#if !defined(VTK_LEGACY_REMOVE) && !defined(VTK_OPENGL2)
//  volumeMapper->SetRequestedRenderModeToRayCastAndTexture();
//#endif  // VTK_LEGACY_REMOVE
  renWin->Render();

  iren->Start();

  return 0;
}
