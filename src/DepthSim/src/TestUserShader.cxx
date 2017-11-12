/*=========================================================================

  Program:   Visualization Toolkit

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtk-8.0/vtkCamera.h"
#include "vtk-8.0/vtkRenderer.h"
#include "vtk-8.0/vtkRenderWindow.h"
#include "vtk-8.0/vtkActor.h"
#include "vtk-8.0/vtkOpenGLPolyDataMapper.h"
#include "vtk-8.0/vtkPLYReader.h"
#include "vtk-8.0/vtkNew.h"
#include "vtk-8.0/vtkProperty.h"
#include "vtk-8.0/vtkPolyDataNormals.h"

//#include "vtk-8.0/vtkRegressionTestImage.h"
//#include "vtk-8.0/vtkTestUtilities.h"
#include "vtk-8.0/vtkRenderWindowInteractor.h"
//----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    vtkNew<vtkOpenGLPolyDataMapper> mapper;

  std::cout<<"hello1";
  vtkNew<vtkActor> actor;
    std::cout<<"hello2";
  vtkNew<vtkRenderer> renderer;
    std::cout<<"hello3";

  renderer->SetBackground(0.0, 0.0, 0.0);
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetSize(400, 400);
  renderWindow->AddRenderer(renderer.Get());
  renderer->AddActor(actor.Get());
  vtkNew<vtkRenderWindowInteractor>  iren;
  iren->SetRenderWindow(renderWindow.Get());
  std::cout<<"hello4";

  vtkNew<vtkPLYReader> reader;
  reader->SetFileName("/home/drc/Chris/depth_sim/bunny.ply");
  reader->Update();
  vtkNew<vtkPolyDataNormals> norms;
  norms->SetInputConnection(reader->GetOutputPort());
  norms->Update();

  mapper->SetInputConnection(norms->GetOutputPort());
  actor->SetMapper(mapper.Get());
  actor->GetProperty()->SetAmbientColor(0.2, 0.2, 1.0);
  actor->GetProperty()->SetDiffuseColor(1.0, 0.65, 0.7);
  actor->GetProperty()->SetSpecularColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetSpecular(0.5);
  actor->GetProperty()->SetDiffuse(0.7);
  actor->GetProperty()->SetAmbient(0.5);
  actor->GetProperty()->SetSpecularPower(20.0);
  actor->GetProperty()->SetOpacity(1.0);

  // Modify the shader to color based on model normal
  // To do this we have to modify the vertex shader
  // to pass the normal in model coordinates
  // through to the fragment shader. By default the normal
  // is converted to View coordinates and then passed on.
  // We keep that, but add a varying for the original normal.
  // Then we modify the fragment shader to set the diffuse color
  // based on that normal. First lets modify the vertex
  // shader
  mapper->AddShaderReplacement(
    vtkShader::Vertex,
    "//VTK::Normal::Dec", // replace the normal block
    true, // before the standard replacements
    "//VTK::Normal::Dec\n" // we still want the default
    "  varying vec3 myNormalMCVSOutput;\n", //but we add this
    false // only do it once
    );
  mapper->AddShaderReplacement(
    vtkShader::Vertex,
    "//VTK::Normal::Impl", // replace the normal block
    true, // before the standard replacements
    "//VTK::Normal::Impl\n" // we still want the default
    "  myNormalMCVSOutput = normalMC;\n", //but we add this
    false // only do it once
    );

  // now modify the fragment shader
  mapper->AddShaderReplacement(
    vtkShader::Fragment,  // in the fragment shader
    "//VTK::Normal::Dec", // replace the normal block
    true, // before the standard replacements
    "//VTK::Normal::Dec\n" // we still want the default
    "  varying vec3 myNormalMCVSOutput;\n", //but we add this
    false // only do it once
    );
  mapper->AddShaderReplacement(
    vtkShader::Fragment,  // in the fragment shader
    "//VTK::Normal::Impl", // replace the normal block
    true, // before the standard replacements
    "//VTK::Normal::Impl\n" // we still want the default calc
    "  diffuseColor = abs(myNormalMCVSOutput);\n", //but we add this
    false // only do it once
    );

  renderWindow->Render();
  renderer->GetActiveCamera()->SetPosition(-0.2,0.4,1);
  renderer->GetActiveCamera()->SetFocalPoint(0,0,0);
  renderer->GetActiveCamera()->SetViewUp(0,1,0);
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Zoom(1.3);
  renderWindow->Render();


  iren->Start();
  return 0;
}