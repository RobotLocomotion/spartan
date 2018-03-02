import vtk

vert = """
    varying vec3 n;
    varying vec3 l;

    void propFuncVS(void)
    {
        n = normalize(gl_Normal);
        l = vec3(gl_ModelViewMatrix * vec4(n,0));
        gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    }
"""

frag = """
    varying vec3 n;
    varying vec3 l;

    void propFuncFS( void )
    {
        vec3 cl = vec3(.2,0,.5);
        vec3 light = normalize(l.xyz);
        float vdn = light.z;
        cl = round(vdn * 5) / 5 * cl;
        gl_FragColor = vec4(cl*vdn,1);
        if (vdn < 0.3)
        {
            gl_FragColor = vec4(vec3(0),1);
        }
    }
"""

# This creates a donut mesh
sphere = vtk.vtkSuperquadricSource()
sphere.ToroidalOn()
sphere.SetThetaResolution(50)
sphere.SetPhiResolution(50)

# This one loads a skull :)
reader = vtk.vtkSTLReader()
reader.SetFileName("small_data/skull.stl")
normals = vtk.vtkPolyDataNormals()
normals.SetInputConnection(reader.GetOutputPort())

# Choose either the donut or the skull
mapper = vtk.vtkPolyDataMapper()
mapper.SetInputConnection(sphere.GetOutputPort())
# mapper.SetInputConnection(normals.GetOutputPort())

# If you want to set the opacity in the shader to anything lower than 1,
# make sure you SetOpacity to something below 1 too, or it will be treated as opaque
actor = vtk.vtkActor()
actor.SetMapper(mapper)
actor.GetProperty().SetOpacity(1)

# Handle the rendering and interaction
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

# Now let's get down to shader-business...
# First we make a ShaderProgram2 and set it up on a date with the RenderWindow
pgm = vtk.vtkShaderProgram2()
pgm.SetContext(renWin)

# For both the vertex and fragment shader, we need to make a Shader2
# Also set them up with the RenderWindow, by asking the ShaderProgram2 for an introduction
shaderv = vtk.vtkShader2()
shaderv.SetType(vtk.VTK_SHADER_TYPE_VERTEX)
shaderv.SetSourceCode(vert)
shaderv.SetContext(pgm.GetContext())
shaderf = vtk.vtkShader2()
shaderf.SetType(vtk.VTK_SHADER_TYPE_FRAGMENT)
shaderf.SetSourceCode(frag)
shaderf.SetContext(pgm.GetContext())

# Now we add the shaders to the program
pgm.GetShaders().AddItem(shaderv)
pgm.GetShaders().AddItem(shaderf)

# And tell the actor property that it should totally use this cool program
openGLproperty = actor.GetProperty()
openGLproperty.SetPropProgram(pgm)
openGLproperty.ShadingOn()

# Add the actor and set a nice bg color
ren.AddActor(actor)
ren.SetBackground(0.3, 0, 0.4)
ren.SetBackground2(0.1, 0, 0.2)
ren.SetGradientBackground(1)

iren.Initialize()
ren.GetActiveCamera().SetPosition(0, -1, 0)
ren.GetActiveCamera().SetViewUp(0, 0, 1)
ren.ResetCamera()
renWin.Render()
iren.Start()