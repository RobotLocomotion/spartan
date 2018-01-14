#pragma once

#include <Eigen/Dense>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCleanPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIdList.h>
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkObjectFactory.h>
#include <vtkPLYReader.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkProperty.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtksys/SystemTools.hxx>

// Generic loader that loads common model formats into
// a vtkPolyData object.
static vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileName) {
  vtkSmartPointer<vtkPolyData> polyData;
  std::string extension =
      vtksys::SystemTools::GetFilenameExtension(std::string(fileName));
  if (extension == ".ply") {
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  } else if (extension == ".vtp") {
    vtkSmartPointer<vtkXMLPolyDataReader> reader =
        vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  } else if (extension == ".obj") {
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  } else if (extension == ".stl") {
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(fileName);
    reader->Update();
    polyData = reader->GetOutput();
  }
  return polyData;
}

static void WritePolyData(const vtkSmartPointer<vtkPolyData> polyData,
                          const char* filename) {
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(filename);
  writer->SetInputData(polyData);
  writer->Write();
}

static vtkSmartPointer<vtkPolyData> DownsampleAndCleanPolyData(
    const vtkSmartPointer<vtkPolyData> polyData, double downsample_spacing) {
  vtkSmartPointer<vtkPolyData> cloudPolyDataDownsampled = polyData;

  vtkSmartPointer<vtkPolyDataPointSampler> pointSampler =
      vtkSmartPointer<vtkPolyDataPointSampler>::New();
  pointSampler->SetDistance(downsample_spacing);
  pointSampler->SetInputData(polyData);
  pointSampler->Update();
  cloudPolyDataDownsampled = pointSampler->GetOutput();

  vtkSmartPointer<vtkCleanPolyData> pointCleaner =
      vtkSmartPointer<vtkCleanPolyData>::New();
  pointCleaner->SetToleranceIsAbsolute(true);
  pointCleaner->SetAbsoluteTolerance(downsample_spacing);
  pointCleaner->SetInputData(cloudPolyDataDownsampled);
  pointCleaner->Update();
  return pointCleaner->GetOutput();
}

static Eigen::Matrix3Xd LoadAndDownsamplePolyData(
    const std::string fileName, double downsample_spacing = -1.0) {
  vtkSmartPointer<vtkPolyData> cloudPolyData = ReadPolyData(fileName.c_str());
  cout << "Loaded " << cloudPolyData->GetNumberOfPoints() << " points from "
       << fileName << endl;

  if (downsample_spacing > 0.0) {
    cloudPolyData =
        DownsampleAndCleanPolyData(cloudPolyData, downsample_spacing);
    cout << "Downsampled to " << cloudPolyData->GetNumberOfPoints() << " points"
         << endl;
  }

  Eigen::Matrix3Xd out_pts(3, cloudPolyData->GetNumberOfPoints());
  for (int i = 0; i < cloudPolyData->GetNumberOfPoints(); i++) {
    out_pts(0, i) = cloudPolyData->GetPoint(i)[0];
    out_pts(1, i) = cloudPolyData->GetPoint(i)[1];
    out_pts(2, i) = cloudPolyData->GetPoint(i)[2];
  }
  return out_pts;
}

// Shamelessly taken from vtkPCLConversions.h in Director.
vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts) {
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts * 2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i) {
    ids[i * 2] = 1;
    ids[i * 2 + 1] = i;
  }

  vtkSmartPointer<vtkCellArray> cellArray =
      vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

static Eigen::Matrix3Xd Matrix3XdFromPolyData(const vtkSmartPointer<vtkPolyData> cloudPolyData){
  Eigen::Matrix3Xd out_pts(3, cloudPolyData->GetNumberOfPoints());
  for (int i = 0; i < cloudPolyData->GetNumberOfPoints(); i++) {
    out_pts(0, i) = cloudPolyData->GetPoint(i)[0];
    out_pts(1, i) = cloudPolyData->GetPoint(i)[1];
    out_pts(2, i) = cloudPolyData->GetPoint(i)[2];
  }
  return out_pts; 
}

static vtkSmartPointer<vtkPolyData> PolyDataFromMatrix3Xd(
    Eigen::Matrix3Xd input) {
  vtkIdType nr_points = input.cols();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  for (vtkIdType i = 0; i < nr_points; ++i) {
    float point[3] = {(float)input(0, i), (float)input(1, i),
                      (float)input(2, i)};
    points->SetPoint(i, point);
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}

static void AddColorToPolyData(const vtkSmartPointer<vtkPolyData> polyData,
                               std::vector<std::vector<double>> colors) {
  vtkSmartPointer<vtkUnsignedCharArray> colors_vtk =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors_vtk->SetNumberOfComponents(3);
  colors_vtk->SetName("Colors");

  // Convert colors from floats on [0, 1] to chars on [0, 255]
  for (int i = 0; i < colors.size(); i++) {
    unsigned char color[3];
    for (int j = 0; j < 3; j++) {
      color[j] = colors[i][j] * 255;
    }
    colors_vtk->InsertNextTypedTuple(color);
  }

  polyData->GetPointData()->SetScalars(colors_vtk);
}