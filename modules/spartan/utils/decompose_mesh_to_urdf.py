'''

Uses v-hacd (https://github.com/kmammou/v-hacd)
wrapped by trimesh (https://github.com/mikedh/trimesh)
to perform convex decomposition of a supplied mesh, and
write the results to a set of .obj meshes, plus a URDF
binding them all together into a single rigid body.

Invocation:

python -m spartan.utils.decompose_mesh_to_urdf \
  <input mesh> <input mesh mass> <output directory name>
  <whether to visualize?> <optional vhacd arguments>

Optional vhacd arguments are currently:
  --maxhulls
  --maxNumVerticesPerCH

'''

import argparse
import sys

import trimesh
from trimesh.io.urdf import *

def do_convex_decomposition_to_urdf(obj_filename, obj_mass, output_directory, do_visualization=False, **kwargs):
  mesh = trimesh.load(obj_filename)
  
  if (do_visualization):
    print("\n\nShowing input mesh...")
    mesh.show()

  mesh.density = obj_mass / mesh.volume
  decomposed_mesh = export_urdf(mesh, output_directory, **kwargs)
  

  print("Input mesh had ", len(mesh.faces), " faces and ", len(mesh.vertices), " verts")
  print("Output mesh has ", len(decomposed_mesh.faces), " faces and ", len(decomposed_mesh.vertices), " verts")

  if (do_visualization):
    print("\n\nShowing output mesh...")
    decomposed_mesh.show()

if __name__ == "__main__":
  trimesh.util.attach_to_log()

  parser = argparse.ArgumentParser(description = "")
  parser.add_argument('filename', help="Mesh file to decompose.")
  parser.add_argument('mass', help="Mass of the original mesh file.", type=float)
  parser.add_argument('output_directory', help="Output directory for files and urdf.", default="output")
  parser.add_argument('visualize', help="Whether to visualize", default=False, type=bool)
  
  # VHACD forwarded argumnets
  vhacd_forwared_args = ["maxhulls", "maxNumVerticesPerCH"]
  parser.add_argument('--maxhulls', help="Maximum convex hulls to produce (default is no limit)", default=None, type=int)
  parser.add_argument('--maxNumVerticesPerCH', help="Maximum vertices per convex hull (default 64, range 4-1024)", default=None, type=int)

  args = parser.parse_args()

  vhacd_forwarded_arg_vals = {}
  for arg in vhacd_forwared_args:
    if getattr(args, arg) is not None:
      vhacd_forwarded_arg_vals[arg] = getattr(args, arg)

  do_convex_decomposition_to_urdf(args.filename, args.mass, args.output_directory, args.visualize, **vhacd_forwarded_arg_vals)