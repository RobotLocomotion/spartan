'''
Usage:

directorPython convertPlyToVtp.py <path/to/data.vtp> <spacing>

This script will read the vtp file and save a vtp file.
It'll resample the input VTP polyData based on the given
absolute uniform spacing, and if bounds are provided,
prune the point cloud to the given absolute bounding box.
'''

import os
from director import ioUtils
from director import segmentation
from director import vtkNumpy as vnp
import vtk

if __name__ == '__main__':

    filename = sys.argv[1]
    gridsize = float(sys.argv[2])

    if len(sys.argv) == 9:
        nx = float(sys.argv[3])
        px = float(sys.argv[4])
        ny = float(sys.argv[5])
        py = float(sys.argv[6])
        nz = float(sys.argv[7])
        pz = float(sys.argv[8])


    polyData = ioUtils.readPolyData(filename)

    print 'input polydata has ', polyData.GetNumberOfPoints(), ' points and bounds ', polyData.GetBounds()

    if len(sys.argv) == 9:
        polyData = segmentation.cropToBounds(polyData, vtk.vtkTransform(), [[nx,px],[ny,py],[nz,pz]])

    sampler = vtk.vtkPolyDataPointSampler()
    sampler.SetDistance(gridsize)
    sampler.SetInput(polyData)
    sampler.Update()
    sampled_data = sampler.GetOutput()

    cleaner = vtk.vtkCleanPolyData()
    cleaner.SetToleranceIsAbsolute(1)
    cleaner.SetAbsoluteTolerance(gridsize)
    cleaner.SetInput(sampled_data)
    cleaner.Update()
    output_data = cleaner.GetOutput()



    print 'downsampled at gridsize ', gridsize, ' to get ', output_data.GetNumberOfPoints(), ' points'

    outputFilename = os.path.splitext(filename)[0] + '_' + str(output_data.GetNumberOfPoints()) + 'pts.vtp'

    print 'writing:', outputFilename, 
    ioUtils.writePolyData(output_data, outputFilename)



