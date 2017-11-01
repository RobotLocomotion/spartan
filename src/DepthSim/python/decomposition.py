from bell2014.solver import IntrinsicSolver
from bell2014.input import IntrinsicInput
from bell2014.params import IntrinsicParameters
from bell2014 import image_util
import glob,os

def solve_intrinsics(dir):
	os.chdir(dir)
	for file in glob.glob("*rgb.png"):
		print(file)
		input = IntrinsicInput.from_file(
			file
		)
		solver = IntrinsicSolver(input,IntrinsicParameters())
		r, s, decomposition = solver.solve()
		name = file.split(".")[0]
		image_util.save(dir+"/"+name+"_r.png", r, rescale=True)
		image_util.save(dir+"/"+name+"_s.png", s, rescale=True)

base_dir = "/media/drc/DATA/chris_labelfusion/logs_test/2017-06-16-15/images"
solve_intrinsics(base_dir)