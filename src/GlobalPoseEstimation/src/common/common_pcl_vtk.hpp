#pragma once

#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIdList.h>
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

#include "common_pcl.hpp"
#include "common_vtk.hpp"

//----------------------------------------------------------------------------
static vtkSmartPointer<vtkPolyData>
PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) {
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  vtkNew<vtkUnsignedCharArray> rgbArray;
  rgbArray->SetName("rgb_colors");
  rgbArray->SetNumberOfComponents(3);
  rgbArray->SetNumberOfTuples(nr_points);

  if (cloud->is_dense) {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y,
                        cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g,
                                cloud->points[i].b};
      points->SetPoint(i, point);
      rgbArray->SetTupleValue(i, color);
    }
  } else {
    vtkIdType j = 0; // true point index
    for (vtkIdType i = 0; i < nr_points; ++i) {
      // Check if the point is invalid
      if (!pcl_isfinite(cloud->points[i].x) ||
          !pcl_isfinite(cloud->points[i].y) ||
          !pcl_isfinite(cloud->points[i].z))
        continue;

      float point[3] = {cloud->points[i].x, cloud->points[i].y,
                        cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g,
                                cloud->points[i].b};
      points->SetPoint(j, point);
      rgbArray->SetTupleValue(j, color);
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints(nr_points);
    rgbArray->SetNumberOfTuples(nr_points);
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->GetPointData()->AddArray(rgbArray.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}

static vtkSmartPointer<vtkPolyData>
PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  if (cloud->is_dense) {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y,
                        cloud->points[i].z};
      points->SetPoint(i, point);
    }
  } else {
    vtkIdType j = 0; // true point index
    for (vtkIdType i = 0; i < nr_points; ++i) {
      // Check if the point is invalid
      if (!pcl_isfinite(cloud->points[i].x) ||
          !pcl_isfinite(cloud->points[i].y) ||
          !pcl_isfinite(cloud->points[i].z))
        continue;

      float point[3] = {cloud->points[i].x, cloud->points[i].y,
                        cloud->points[i].z};
      points->SetPoint(j, point);
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints(nr_points);
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}

//----------------------------------------------------------------------------
static pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudFromPolyData(vtkPolyData *polyData) {
  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);

  if (!numberOfPoints) {
    return cloud;
  }

  vtkFloatArray *floatPoints =
      vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
  vtkDoubleArray *doublePoints =
      vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());
  assert(floatPoints || doublePoints);

  if (floatPoints) {
    float *data = floatPoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      cloud->points[i].x = data[i * 3];
      cloud->points[i].y = data[i * 3 + 1];
      cloud->points[i].z = data[i * 3 + 2];
    }
  } else if (doublePoints) {
    double *data = doublePoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      cloud->points[i].x = data[i * 3];
      cloud->points[i].y = data[i * 3 + 1];
      cloud->points[i].z = data[i * 3 + 2];
    }
  }

  return cloud;
}

//----------------------------------------------------------------------------
static pcl::PointCloud<pcl::PointNormal>::Ptr
PointCloudFromPolyDataWithNormals(vtkPolyData *polyData) {
  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointNormal>);
  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);

  if (!numberOfPoints) {
    return cloud;
  }

  // Get points
  vtkFloatArray *floatPoints =
      vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
  vtkDoubleArray *doublePoints =
      vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());
  assert(floatPoints || doublePoints);

  if (floatPoints) {
    float *data = floatPoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      cloud->points[i].x = data[i * 3];
      cloud->points[i].y = data[i * 3 + 1];
      cloud->points[i].z = data[i * 3 + 2];
    }
  } else if (doublePoints) {
    double *data = doublePoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      cloud->points[i].x = data[i * 3];
      cloud->points[i].y = data[i * 3 + 1];
      cloud->points[i].z = data[i * 3 + 2];
    }
  }

  vtkFloatArray *floatNormals =
      vtkFloatArray::SafeDownCast(polyData->GetPointData()->GetNormals());
  vtkDoubleArray *doubleNormals =
      vtkDoubleArray::SafeDownCast(polyData->GetPointData()->GetNormals());

  if (!doubleNormals && !floatNormals) {
    printf("No normals available in this polyData!\n");
    return cloud;
  }

  if (floatNormals) {
    printf("Using float normals\n");
    float *data = floatNormals->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      cloud->points[i].normal_x = data[i * 3];
      cloud->points[i].normal_y = data[i * 3 + 1];
      cloud->points[i].normal_z = data[i * 3 + 2];
    }
  } else if (doubleNormals) {
    printf("Using double normals\n");
    double *data = doubleNormals->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i) {
      cloud->points[i].normal_x = data[i * 3];
      cloud->points[i].normal_y = data[i * 3 + 1];
      cloud->points[i].normal_z = data[i * 3 + 2];
    }
  }

  return cloud;
}

static Eigen::Matrix<double, 6, -1>
LoadMatrixWithVTKWithNormals(const std::string filename) {
    // Load in the scene cloud
  vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(filename.c_str());
  cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from " << filename << endl;

  pcl::PointCloud<pcl::PointNormal>::Ptr sceneCloud(new pcl::PointCloud<pcl::PointNormal>());
  if (cloudPolyData->GetPointData()->GetNormals()) {
    sceneCloud = PointCloudFromPolyDataWithNormals(cloudPolyData);
  } else {
    printf("No normals available in input scene file... generating own.\n");
    printf("Note: it's possible that VTK just isn't loading your normals!\n");
    printf("The STL reader is known not to read normals. Try using meshlab to convert to OBJ.\n");
    printf("Regenerate your vertex normals while you're at it :)\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudWithoutNormals =
        PointCloudFromPolyData(cloudPolyData);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sceneCloudWithoutNormals);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr sceneCloudNormals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.setViewPoint (0.0, 0.0, 0.0);
    // Compute the features
    ne.compute (*sceneCloudNormals);

    assert(sceneCloudWithoutNormals->size() == sceneCloudNormals->size());
    sceneCloud->points.resize(sceneCloudWithoutNormals->size());
    printf("Merging clouds...\n");
    for (size_t i = 0; i < sceneCloud->points.size(); i++) {
        sceneCloud->points[i].x = sceneCloudWithoutNormals->points[i].x;
        sceneCloud->points[i].y = sceneCloudWithoutNormals->points[i].y;
        sceneCloud->points[i].z = sceneCloudWithoutNormals->points[i].z;
        sceneCloud->points[i].normal_x = sceneCloudNormals->points[i].normal_x;
        sceneCloud->points[i].normal_y = sceneCloudNormals->points[i].normal_y;
        sceneCloud->points[i].normal_z = sceneCloudNormals->points[i].normal_z;
    }
  }
  printf("Done and loaded.\n");

  return convertPCLPointNormalToMatrix6Xd(sceneCloud);
}