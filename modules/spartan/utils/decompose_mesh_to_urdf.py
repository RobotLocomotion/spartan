
import argparse
import sys

import trimesh
from trimesh.io.urdf import *

def do_convex_decomposition_to_urdf(obj_filename, obj_mass, output_directory, do_visualization=False):
  mesh = trimesh.load(obj_filename)
  mesh.density = obj_mass / mesh.volume
  decomposed_mesh = export_urdf(mesh, output_directory)
  if (do_visualization):
    decomposed_mesh.show()

if __name__ == "__main__":
  trimesh.util.attach_to_log()

  parser = argparse.ArgumentParser(description = "")
  parser.add_argument('filename', help="Mesh file to decompose.")
  parser.add_argument('mass', help="Mass of the original mesh file.", type=float)
  parser.add_argument('output_directory', help="Output directory for files and urdf.", default="output")
  parser.add_argument('visualize', help="Whether to visualize", default=False, type=bool)

  args = parser.parse_args()

  do_convex_decomposition_to_urdf(args.filename, args.mass, args.output_directory, args.visualize)