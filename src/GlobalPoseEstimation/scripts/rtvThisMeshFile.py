import os
from director import ioUtils
from director import segmentation
from director import vtkNumpy as vnp
from director import transformUtils
from director import viewerclient
import vtk
import re
import yaml
import numpy as np

# Run with directorPython!

class MeshFile(viewerclient.BaseGeometry):
    __slots__ = ["filename", "scale"]
    def __init__(self, filename, scale=1.0):
        self.scale = scale
        self.filename = filename

    def serialize(self):
        return {
            "type": "mesh_file",
            "scale": [self.scale, self.scale, self.scale],
            "filename": self.filename
        }

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print "Usage: directorPython <script name> <mesh file>"
        exit(1)

    file = sys.argv[1]


    # We can provide an initial path if we want
    vis = viewerclient.Visualizer(path="/visualize_result/")

    # Start a thread to handle responses from the viewer. Doing this enables
    # the automatic reloading of missing geometry if the viewer is restarted.
    vis.start_handler()

    est_geom = viewerclient.GeometryData(MeshFile(os.path.abspath(file)))
    vis[file].setgeometry(est_geom)








