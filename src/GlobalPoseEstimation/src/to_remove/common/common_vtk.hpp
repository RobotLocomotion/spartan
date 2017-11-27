#pragma once

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIdList.h>
#include <vtkIntArray.h>
#include <vtkObjectFactory.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtksys/SystemTools.hxx>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

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

// Generic loader that loads common model formats into
// a vtkPolyData object.
static vtkSmartPointer<vtkPolyData> ReadPolyData(const char *fileName)
{
  vtkSmartPointer<vtkPolyData> polyData;
  std::string extension = vtksys::SystemTools::GetFilenameExtension(std::string(fileName));
  if (extension == ".ply")
  {
    vtkSmartPointer<vtkPLYReader> reader =
      vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName (fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".vtp")
  {
    vtkSmartPointer<vtkXMLPolyDataReader> reader =
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName (fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".obj")
  {
    vtkSmartPointer<vtkOBJReader> reader =
      vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName (fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  else if (extension == ".stl")
  {
    vtkSmartPointer<vtkSTLReader> reader =
      vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName (fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  return polyData;
}

static void WritePolyData(const vtkSmartPointer<vtkPolyData> polyData, const char * filename) 
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =  
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename);
  writer->SetInput(polyData);
  writer->Write();
}

static Eigen::Matrix3Xd LoadAndDownsamplePolyData(const std::string fileName, double downsample_spacing = -1.0)
{
  vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(fileName.c_str());
  cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from " << fileName << endl;

  vtkSmartPointer<vtkPolyData> cloudPolyDataOut;

  if (downsample_spacing <= 0.0){
    cloudPolyDataOut = cloudPolyData;
    printf("... and not downsampling.\n");
  } else {
    vtkSmartPointer<vtkPolyDataPointSampler> pointSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
    pointSampler->SetDistance(downsample_spacing);
    pointSampler->SetInput(cloudPolyData);
    pointSampler->Update();
    vtkSmartPointer<vtkPolyData> cloudPolyDataDownsampled = pointSampler->GetOutput();

    printf("sampled but not cleaned\n");
    vtkSmartPointer<vtkCleanPolyData> pointCleaner = vtkSmartPointer<vtkCleanPolyData>::New();
    pointCleaner->SetToleranceIsAbsolute(true);
    pointCleaner->SetAbsoluteTolerance(downsample_spacing);
    pointCleaner->SetInput(cloudPolyDataDownsampled);
    pointCleaner->Update();
    cloudPolyDataOut = pointCleaner->GetOutput();

    cout << "Downsampled to " << cloudPolyDataOut->GetNumberOfPoints() << " points" << endl;
  }

  Eigen::Matrix3Xd out_pts(3, cloudPolyDataOut->GetNumberOfPoints());
  for (int i=0; i<cloudPolyDataOut->GetNumberOfPoints(); i++){
    out_pts(0, i) = cloudPolyDataOut->GetPoint(i)[0];
    out_pts(1, i) = cloudPolyDataOut->GetPoint(i)[1];
    out_pts(2, i) = cloudPolyDataOut->GetPoint(i)[2];
  }
  return out_pts;
}


//----------------------------------------------------------------------------
static vtkSmartPointer<vtkPolyData> PolyDataFromMatrix3Xd(Eigen::Matrix3Xd input)
{
  vtkIdType nr_points = input.cols();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);


  for (vtkIdType i = 0; i < nr_points; ++i) {
    float point[3] = {(float)input(0, i),
                      (float)input(1, i),
                      (float)input(2, i)};
    points->SetPoint(i, point);
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}