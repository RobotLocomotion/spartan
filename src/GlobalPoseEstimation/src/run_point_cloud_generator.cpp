/*
 */
#include <libgen.h>

#include "yaml-cpp/yaml.h"
#include "common_utils/math_utils.h"
#include "common_utils/system_utils.h"
#include "common_utils/vtk_utils.h"

#include "point_cloud_generator.hpp"
#include "RemoteTreeViewerWrapper.hpp"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  srand(getUnixTime());

  if (argc < 2){
    printf("Use: run_point_cloud_generator <config file> <optional output_file> <optional ground_truth_output_file\n");
    exit(-1);
  }

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Point Cloud Generator Interface" << asctime(curtime);
  cout << "Config file " << string(argv[1]) << endl;
  if (argc > 2)
    cout << "Output file " << string(argv[2]) << endl;
  cout << "***************************" << endl << endl;
  if (argc > 3)
    cout << "Ground truth output file " << string(argv[3]) << endl;
  cout << "***************************" << endl << endl;

  // Bring in config file
  string yamlString = string(argv[1]);
  YAML::Node config = YAML::LoadFile(yamlString);

  // Set up a point cloud generator
  if (config["point_cloud_generator_options"] == NULL){
    runtime_error("Config needs a point cloud generator option set.");
  }
  // Warning: yucky string->char* conversion here that doesn't work if the base string
  // gets modified.
  PointCloudGenerator pcg(config["point_cloud_generator_options"], dirname(&yamlString[0]));

  // Generate a point set
  Matrix3Xd scene_pts = pcg.samplePointCloud();

  // Visualize the results using the drake visualizer.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  rm.publishPointCloud(scene_pts, {"scene_pts"}, {{0.1, 1.0, 0.1}});
  rm.publishRigidBodyTree(pcg.get_robot(), pcg.get_q_robot(), Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});

  if (argc > 2){
    string outputFilename = string(argv[2]);
    WritePolyData(PolyDataFromMatrix3Xd(scene_pts), outputFilename.c_str());
    cout << "Saved " << scene_pts.cols() << " points to " << outputFilename << endl;
  }

  if (argc > 3){
    // Spit out a ground truth annotation file as well
    // This is very simple... just grab the models node of the point cloud
    // config and spit that out on its own.
    ofstream fout(argv[3]);
    fout << config["point_cloud_generator_options"]["models"];
    fout.close();
  }
  return 0;
}
