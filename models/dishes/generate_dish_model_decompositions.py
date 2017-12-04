import sys
from spartan.utils.decompose_mesh_to_urdf import do_convex_decomposition_to_urdf

if len(sys.argv) != 2:
	print "Please say which model to regenerate (or `all`)"
	exit(0)

do_all = (sys.argv[1] == "all")
do_viz = False

if do_all or sys.argv[1] == "plate_11in":
	do_convex_decomposition_to_urdf("meshes/visual/plate_11in.obj", \
		0.282, \
		"plate_11in_decomp", \
		do_visualization = do_viz, \
		scale=0.001, \
		color=[0.5, 0.9, 0.5], \
		**{"maxhulls": 8, "maxNumVerticesPerCH": 15})

if do_all or sys.argv[1] == "plate_8p5in":
	do_convex_decomposition_to_urdf("meshes/visual/plate_8p5in.obj", \
		0.180, \
		"plate_8p5in_decomp", \
		do_visualization = do_viz, \
		scale=0.001, \
		color=[0.9, 0.5, 0.5], \
		**{"maxhulls": 8, "maxNumVerticesPerCH": 15})

if do_all or sys.argv[1] == "bowl_6p25in":
	do_convex_decomposition_to_urdf("meshes/visual/bowl_6p25in.obj", \
		0.140, \
		"bowl_6p25in_decomp", \
		do_visualization = do_viz, \
		scale=0.001, \
		color=[0.5, 0.5, 0.9], \
		**{"maxhulls": 12, "maxNumVerticesPerCH": 15})

#if do_all or sys.argv[1] == "dish_rack":
#	do_convex_decomposition_to_urdf("meshes/visual/dish_rack.obj", \
#		0.140, \
#		"dish_rack_decomp", \
#		do_visualization = do_viz, \
#		scale=0.001, \
#		**{"maxhulls": 12, "maxNumVerticesPerCH": 15})