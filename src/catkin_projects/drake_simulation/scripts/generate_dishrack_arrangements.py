import argparse
import math
import numpy as np
import os
import time
import yaml

# For projection into nonpenetration
# (pre-processing for simulation)
import pydrake
from pydrake.solvers import ik

# Eventually, for visualization,
# though not used yet.
from director import viewerclient


models = {
    "plate_11in": "drake/../../../../../models/dishes/plate_11in_decomp/plate_11in_decomp.urdf",
    "plate_8p5in": "drake/../../../../../models/dishes/plate_8p5in_decomp/plate_8p5in_decomp.urdf",
    "bowl_6p25in": "drake/../../../../../models/dishes/bowl_6p25in_decomp/bowl_6p25in_decomp.urdf", 
    "dish_rack": "drake/../../../../../models/dishes/dish_rack_simple.urdf"
}


class ObjectInstance:
    def __init__(self, model, q0, fixed=False):
        self.model = model
        self.q0 = q0
        self.fixed = fixed
    def to_dict(self):
        return {"model": str(self.model), 
                "q0": [float(x) for x in self.q0], "fixed": self.fixed}

def load_rbt_from_urdf_rel_drake_root(model_name, rbt, weld_frame = None):
    urdf_filename = models[model_name]
    drake_root = os.getenv("DRAKE_RESOURCE_ROOT")
    urdf_string = open(drake_root + "/" + urdf_filename).read()
    base_dir = os.path.dirname(drake_root + "/" + urdf_filename)
    package_map = pydrake.rbtree.PackageMap()
    floating_base_type = pydrake.rbtree.kRollPitchYaw
    pydrake.rbtree.AddModelInstanceFromUrdfStringSearchingInRosPackages(
        urdf_string,
        package_map,
        base_dir,
        floating_base_type,
        weld_frame,
        rbt)

class DishrackArrangement:
    def __init__(self):
        self.instances = [ObjectInstance("dish_rack", [0, 0, 0, 0, 0, 0], True)]

    def add_instance(self, instance):
        self.instances.append(instance)

    def project_instance_to_nonpenetration(self):
        # Set up a rigidbodytree
        r = pydrake.rbtree.RigidBodyTree()
        q0 = [] 
        world_frame = pydrake.rbtree.RigidBodyFrame("world_frame", r.world(), [0, 0, 0], [0, 0, 0])
        for instance in self.instances:
            if instance.fixed:
                load_rbt_from_urdf_rel_drake_root(instance.model, r, world_frame)
                [q0.append(x) for x in instance.q0]

            else:
                load_rbt_from_urdf_rel_drake_root(instance.model, r)
                [q0.append(x) for x in instance.q0]

        constraints = [
            # Nonpenetration: no two bodies closer than 1e-3
            ik.MinDistanceConstraint(r, 1e-3, list(), set())
        ]
        options = ik.IKoptions(r)
        q0_array = np.array(q0, dtype=np.float64, ndmin=2)[0]
        results = ik.InverseKin(r, q0_array, q0_array, constraints, options)
        q0 = results.q_sol[0]
        info = results.info
        print("Results ", q0, " with info ", info)

        # Remap flattened state back into the individual instance states
        ind = 0
        for instance in self.instances:
            num_states = len(instance.q0)
            print("For instance %s: mapped ", instance.q0, " to ", q0[ind:(ind+num_states)])
            instance.q0 = q0[ind:(ind+num_states)]
            ind += num_states

    def save_to_file(self, filename):
        data = {}
        data["models"] = models
        data["with_ground"] = True
        data["instances"] = []
        for instance in self.instances:
            data["instances"].append(instance.to_dict())
        with open(filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

# Place plates vertically, but with random yaw from [0, pi/2, pi, 3pi/2], 
# within the bounds of the rack
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
    yaw = float(np.random.randint(0, 4))*math.pi/2.
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

    num_dishes = np.random.randint(1, max_num_dishes)

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
    arrangement.save_to_file(args.file_prefix + "1_pre_projection.yaml")
    arrangement.project_instance_to_nonpenetration()
    arrangement.save_to_file(args.file_prefix + "1_post_projection.yaml")



