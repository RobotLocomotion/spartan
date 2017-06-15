'''
Usage:

directorPython convertPolyData.py <in> <out>
'''

import os
from director import ioUtils
from director import segmentation
from director import vtkNumpy as vnp
import vtk

if __name__ == '__main__':

    filename = sys.argv[1]
    outputFilename = sys.argv[2]

    polyData = ioUtils.readPolyData(filename)
    ioUtils.writePolyData(polyData, outputFilename)