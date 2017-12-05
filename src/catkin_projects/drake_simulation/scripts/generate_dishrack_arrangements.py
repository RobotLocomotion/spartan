import argparse
import math
import numpy as np
import pydrake
import time
import yaml

models = {
	"plate_11in": "drake/../../../../../models/dishes/plate_11in_decomp/plate_11in_decomp.urdf",
	"plate_8p5in": "drake/../../../../../models/dishes/plate_8p5in_decomp/plate_8p5in_decomp.urdf",
	"bowl_6p25in": "drake/../../../../../models/dishes/bowl_6p25in_decomp/bowl_6p25in_decomp.urdf",	
	"dish_rack": "drake/../../../../../models/dishes/dish_rack_simple.urdf"
}


class ObjectInstance:
	def __init__(self, name, q0, fixed=False):
		self.name = name
		self.q0 = q0
		self.fixed = fixed
	def to_dict(self):
		return {"model": str(self.name), 
				"q0": [float(x) for x in self.q0], "fixed": self.fixed}

class DishrackArrangement:
	def __init__(self):
		self.instances = [ObjectInstance("dish_rack", [0, 0, 0, 0, 0, 0], True)]

	def add_instance(self, instance):
		self.instances.append(instance)

	def save_to_file(self, filename):
		data = {}
		data["models"] = models
		data["with_ground"] = True
		data["instances"] = []
		for instance in self.instances:
			data["instances"].append(instance.to_dict())
		with open(filename, 'w') as outfile:
			yaml.dump(data, outfile, default_flow_style=False)

# Place plates vertically, but with random yaw, within the bounds
# of the rack
# (Yes this is likely to be bad)
plate_11in_params = {
	"lower_bound_x": 0.017,
	"upper_bound_x": 0.47,
	"lower_bound_y": -0.47,
	"upper_bound_y": 0.017,
	"height": 0.3
}
def place_plate_11in():
	center_location_x = np.random.uniform(plate_11in_params["lower_bound_x"], plate_11in_params["upper_bound_x"])
	center_location_y = np.random.uniform(plate_11in_params["lower_bound_y"], plate_11in_params["upper_bound_y"])
	yaw = np.random.uniform(0, 2*math.pi)
	return (center_location_x, center_location_y, plate_11in_params["height"],
	        0, 0, yaw)




placement_generators = {
	"plate_11in": place_plate_11in
}
# Hand-written data generation script
def generate_dishrack_arrangement(max_num_dishes, allowable_dish_types, seed = None):
	if seed is not None:
		np.random.seed(seed)

	arrangement = DishrackArrangement()

	num_dishes = np.random.randint(0, max_num_dishes)

	for k in range(num_dishes):
		# Pick dish type
		dish_type = allowable_dish_types[np.random.randint(0, len(allowable_dish_types))]
		if dish_type not in placement_generators.keys():
			print("Error: generator not defined for dish type %s" % (dish_type))
			exit(-1)
		# Generate a placement and add it to the arrangement
		arrangement.add_instance(ObjectInstance(dish_type, placement_generators[dish_type](), False))

	return arrangement



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_prefix", help="Prefix of arrangement file to generate.", type=str)
    parser.add_argument("-n", "--max_num_dishes", help="Max # of dishes to generate", type=int, default=20)
    parser.add_argument("-s", "--seed", help="Random seed", type=int)
    args = parser.parse_args()

    arrangement = generate_dishrack_arrangement(args.max_num_dishes, ["plate_11in",], args.seed)
    arrangement.save_to_file(args.file_prefix + "1.yaml")


