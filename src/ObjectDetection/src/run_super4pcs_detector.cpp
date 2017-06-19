/*
 */

#include <string>
#include <stdexcept>
#include <iostream>
#include <random>
#include <unistd.h>
#include <typeinfo>

#include "common/common.hpp"
#include "common/common_rtv.hpp"
#include "common/common_vtk.hpp"
#include "yaml-cpp/yaml.h"

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include "RemoteTreeViewerWrapper.hpp"
#include "Super4PCS/src/super4pcs/shared4pcs.h"

using namespace std;
using namespace Eigen;
using namespace match_4pcs;

typedef vector<Vector3f> Points;
typedef vector<VectorXf> Feature;

int main(int argc, char** argv) {
  srand(0);

  if (argc < 3){
    printf("Use: run_super4pcs_detector <scene cloud, vtp> <model cloud, vtp> <config file> <optional output_file>\n");
    exit(-1);
  }

  string sceneFile = string(argv[1]);
  string modelFile = string(argv[2]);
  string configFile = string(argv[3]);
  string outputFile = argc > 3 ? string(argv[4]) : "";

  time_t _tm =time(NULL );
  struct tm * curtime = localtime ( &_tm );
  cout << "***************************" << endl;
  cout << "***************************" << endl;
  cout << "Super4PCS Object Pose Estimator" << asctime(curtime);
  cout << "Scene file " << sceneFile << endl;
  cout << "Model file " << modelFile << endl;
  cout << "Config file " << configFile << endl;
  if (argc > 3)
    cout << "Output file " << outputFile << endl;
  cout << "***************************" << endl << endl;

  // Bring in config file
  YAML::Node config = YAML::LoadFile(configFile);

  if (config["detector_options"] == NULL){
    runtime_error("Need detector options.");
  }
  auto fgr_config = config["detector_options"];

  // Read in the model.
  vtkSmartPointer<vtkPolyData> modelPolyData = ReadPolyData(modelFile.c_str());
  cout << "Loaded " << modelPolyData->GetNumberOfPoints() << " points from " << modelFile << endl;
  Matrix3Xd model_pts(3, modelPolyData->GetNumberOfPoints());
  for (int i=0; i<modelPolyData->GetNumberOfPoints(); i++){
    model_pts(0, i) = modelPolyData->GetPoint(i)[0];
    model_pts(1, i) = modelPolyData->GetPoint(i)[1];
    model_pts(2, i) = modelPolyData->GetPoint(i)[2];
  }

  // Load in the scene cloud
  vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(sceneFile.c_str());
  cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from " << sceneFile << endl;
  Matrix3Xd scene_pts(3, cloudPolyData->GetNumberOfPoints());
  for (int i=0; i<cloudPolyData->GetNumberOfPoints(); i++){
    scene_pts(0, i) = cloudPolyData->GetPoint(i)[0];
    scene_pts(1, i) = cloudPolyData->GetPoint(i)[1];
    scene_pts(2, i) = cloudPolyData->GetPoint(i)[2];
  }

  // Visualize the scene points and GT, to start with.
  RemoteTreeViewerWrapper rm;
  // Publish the scene cloud
  //rm.publishPointCloud(scene_pts_in, {"scene_pts_loaded"}, {{0.1, 1.0, 0.1}});
  rm.publishPointCloud(scene_pts, {"super4pcs", "scene_pts_downsampled"}, {{0.1, 1.0, 1.0}});
  //rm.publishPointCloud(model_pts_in, {"model_pts"}, {{0.1, 1.0, 1.0}});
  rm.publishPointCloud(model_pts, {"super4pcs", "model_pts_downsampled"}, {{0.1, 1.0, 1.0}});
  //rm.publishRigidBodyTree(robot, q_robot, Vector4d(1.0, 0.6, 0.0, 0.2), {"robot_gt"});

/*
  // Shift centroids to origin
  Vector3d scenePtAvg = scene_pts.rowwise().mean();
  Vector3d modelPtAvg = model_pts.rowwise().mean();
  scene_pts.colwise() -= scenePtAvg;
  model_pts.colwise() -= modelPtAvg;

  // Load model and data point clouds into GoICP
  double max_abs_coeff = fmax(
      fmax(fabs(model_pts.minCoeff()), fabs(model_pts.maxCoeff())),
      fmax(fabs(scene_pts.minCoeff()), fabs(scene_pts.maxCoeff()))
    );
  printf("Max abs coeff %f\n", max_abs_coeff);
  scene_pts /= (2.0*max_abs_coeff);
  model_pts /= (2.0*max_abs_coeff);
*/

  // Flip scene and model so "data" is the smaller cloud
  // (to take advantage of the constant-time factor for
  // the model cloud)
  bool flipped = scene_pts.cols() < model_pts.cols();
  std::vector<Point3D> Q;  
  std::vector<Point3D> P;
  // Load in Super4PCS
  for (int i = 0; i < scene_pts.cols(); i++) {
    Vector3d pt = scene_pts.col(i);
    if (!is_finite(pt)){
      printf("Rejecting scene point %d (%f, %f, %f)\n", i, pt[0], pt[1], pt[2]);
      continue;
    }

    Point3D pt_s(pt);
    //pt_s.set_normal(Point3D(1.0, 0.0, 0.0));
    if (flipped){
      Q.push_back(pt_s);
    } else {
      P.push_back(pt_s);
    }
  }
  for (int i = 0; i < model_pts.cols(); i++) {
    Vector3d pt = model_pts.col(i);
    if (!is_finite(pt)){
      printf("Rejecting model point %d (%f, %f, %f)\n", i, pt[0], pt[1], pt[2]);
      continue;
    }
    
    Point3D pt_m(pt);
    //pt_m.set_normal(Point3D(1.0, 0.0, 0.0));
    if (flipped){
      P.push_back(pt_m);
    } else {
      Q.push_back(pt_m);
    }
  }

/*
  // Un-scale the point clouds
  scene_pts *= 2.0*max_abs_coeff;
  model_pts *= 2.0*max_abs_coeff;

  // And un-center them
  scene_pts.colwise() += scenePtAvg;
  model_pts.colwise() += modelPtAvg;
*/

  /*
  // The delta for the LCP (see the paper).
  double delta = 5.0;
  // Estimated overlap between P and Q. This is the fraction of points in P that
  // may have corresponding point in Q. It's being used to estimate the number
  // of RANSAC iterations needed to guarantee small failure probability.
  double overlap_estimation = 0.2;
  // Maximum normal difference.
  double max_normal_difference = 10.0;
  // Maximum translation distance.
  double max_translation_distance = kLargeNumber;
  // Maximum rotation angle.
  double max_angle = kLargeNumber;
  // Maximum color RGB distance between corresponding vertices.
  double max_color_distance = 100.0;
  // Threshold on the value of the target function (LCP, see the paper).
  // It is used to terminate the process once we reached this value.
  double terminate_threshold = 1.0;
  // The number of points in the sample. We sample this number of points
  // uniformly from P and Q.
  int sample_size = 200;
  // Maximum time we allow the computation to take. This makes the algorithm
  // an ANY TIME algorithm that can be stopped at any time, producing the best
  // solution so far.
  int max_time_seconds = 60;
  */
  Match4PCSOptions s4pcs_opts;
  s4pcs_opts.delta = 0.001;
  s4pcs_opts.sample_size = 500;
  s4pcs_opts.overlap_estimation = 0.3;
  s4pcs_opts.max_color_distance = kLargeNumber;
  s4pcs_opts.max_normal_difference = kLargeNumber;

  MatchSuper4PCS matcher(s4pcs_opts);


  // Computes an approximation of the best LCP (directional) from Q to P
  // and the rigid transformation that realizes it. The input sets may or may
  // not contain normal information for any point.
  // @param [in] P The first input set.
  // @param [in] Q The second input set.
  // as a fraction of the size of P ([0..1]).
  // @param [out] transformation Rigid transformation matrix (4x4) that brings
  // Q to the (approximate) optimal LCP.
  // @return the computed LCP measure.
  // The method updates the coordinates of the second set, Q, applying
  // the found transformation.
  cv::Mat tf;

  printf("Running Super4PCS on point vectors of size %lu and %lu\n", P.size(), Q.size());
  clock_t clockBegin = clock();
  
  matcher.ComputeTransformation(P, &Q, &tf);

  clock_t clockEnd = clock();
  double time = (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC;
  cout << "Finished in " << time << endl;
  Affine3d est_tf;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      est_tf.matrix()(i, j) = tf.at<double>(i, j);
    }
  }
  if (!flipped) {
    //est_tf.translation() += (scenePtAvg - est_tf.rotation() * modelPtAvg);
    est_tf = est_tf.inverse();
  } else {
    //est_tf.translation() += (modelPtAvg - est_tf.rotation() * scenePtAvg);
  }

  // Publish the transformed scene point cloud (I'm transforming the scene because
  // the model is usually better centered )
  cout << "Est tf " << est_tf.matrix() << endl;
  cout << "Est tf inv" << est_tf.inverse().matrix() << endl;

  publishErrorColorCodedPointCloud(est_tf * scene_pts, model_pts, "super4pcs");

  return 0;
}
