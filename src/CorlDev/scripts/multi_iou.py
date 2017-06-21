import os
from computeIntersectionOverUnion import computeIOUfile

dir_full_path = os.getcwd()


for filename in sorted(os.listdir(dir_full_path)):
	print filename
	split_filename = filename.split("_")
	if len(split_filename) < 2:
		continue

	# look for labels
	if split_filename[1].startswith("labels"):
		print "found labels"
		img_num = split_filename[0]
		label_file = filename
		continue

	# look for predlabels
	if split_filename[1].startswith("predlabels"):
		print "found predlabels"
		if split_filename[0] != img_num:
			print "error: not matching"
			quit()
		print computeIOUfile(label_file, filename)

		