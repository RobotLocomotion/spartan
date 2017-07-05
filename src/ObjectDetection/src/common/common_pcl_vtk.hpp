#pragma once

#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>


#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIdList.h>
#include <vtkIntArray.h>
#include <vtkObjectFactory.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtksys/SystemTools.hxx>

// Shamelessly taken from vtkPCLConversions.h in Director, which isn't installed in
// a way that I can use it. That should be fixed and this should be eliminated.

vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}


//----------------------------------------------------------------------------
static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  vtkNew<vtkUnsignedCharArray> rgbArray;
  rgbArray->SetName("rgb_colors");
  rgbArray->SetNumberOfComponents(3);
  rgbArray->SetNumberOfTuples(nr_points);


  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b}; 
      points->SetPoint(i, point);
      rgbArray->SetTupleValue(i, color);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) || 
          !pcl_isfinite (cloud->points[i].y) || 
          !pcl_isfinite (cloud->points[i].z))
        continue;

      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
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

static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
      points->SetPoint(i, point);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) || 
          !pcl_isfinite (cloud->points[i].y) || 
          !pcl_isfinite (cloud->points[i].z))
        continue;

      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
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
static pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFromPolyData(vtkPolyData* polyData)
{
  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);

  if (!numberOfPoints)
    {
    return cloud;
    }

  vtkFloatArray* floatPoints = vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
  vtkDoubleArray* doublePoints = vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());
  assert(floatPoints || doublePoints);

  if (floatPoints)
    {
    float* data = floatPoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
      cloud->points[i].x = data[i*3];
      cloud->points[i].y = data[i*3+1];
      cloud->points[i].z = data[i*3+2];
      }
    }
  else if (doublePoints)
    {
    double* data = doublePoints->GetPointer(0);
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
      cloud->points[i].x = data[i*3];
      cloud->points[i].y = data[i*3+1];
      cloud->points[i].z = data[i*3+2];
      }
    }

  return cloud;
}
