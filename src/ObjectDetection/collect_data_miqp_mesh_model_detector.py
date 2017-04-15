#!/usr/bin/env python

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import datetime
import signal

# Make sure you have a drakevisualizer open when running this, so you can see
# the vis output.

def get_miqp_mesh_model_detector_default_options():
  data = dict(
    detector_options = dict(
      detector_type = "body_to_world_transforms",
      #detector_type = "body_to_world_transforms_with_sampled_model_points",

      # options of rot constraints:
      # 0: constrained to ground truth
      # 1: unconstrained
      # 2: columnwise and row-wise L1-norm >= 1
      # 3: McCormick quaternion
      # 4: McCormick directly on rotmat
      # 5: Conservative RPY limits
      rotation_constraint = 0,
      rotation_constraint_num_faces = 2,
      
      allow_outliers = True,
      phi_max = 0.01,
      use_initial_guess = False,
      corruption_amount = 0.0,
      init_guess_rand_seed = -1,

      downsample_to_this_many_points = 50,
      add_this_many_outliers = 0,
      scene_point_additive_noise = 0.00,
      scene_point_rand_seed = 0,
      model_sample_rays = 60,
      model_point_rand_seed = -1,
      
      ICP_use_as_heuristic = False,
      ICP_prior_weight = 0.05,
      ICP_outlier_rejection_proportion = 10,
      ICP_max_iters = 1000,

      gurobi_int_options = dict(
        Threads = 10
      ),

      gurobi_float_options = dict(
        TimeLimit = 120,
        FeasRelaxBigM = 2,
        MIPGapAbs = 0.001, 
        MIPGap = 0.05,
        Heuristics = 0.1
      ),

      models = [
        dict(
          #urdf = "models/urdf/cubes_jointed.urdf",
          urdf = "models/urdf/cube.urdf",
          #q0 = [0.1, 0, 0.5, 0.9, 0.5, 0.2, 0.2]
          q0 = [0.1, 0, 0.5, 0.9, 0.5, 0.2]
        )
      ]
    )
  )
  return data
     

def write_miqp_mesh_model_detector_config(outfilename, options):
  with open(outfilename, 'w') as outfile:
    yaml.dump(options, outfile, default_flow_style=False)


''' Plotter helper '''
def update_plots(outliers, times, objectives, bounds):
    plt.subplot(4, 1, 1)
    plt.plot(outliers, times, 'ro')
    plt.ylabel("Solve Times")

    plt.subplot(4, 1, 2)
    plt.plot(outliers, objectives, 'go')
    plt.ylabel("Solve Objective")

    plt.subplot(4, 1, 3)
    plt.plot(outliers, bounds, 'go')
    plt.ylabel("Bounds")


    plt.subplot(4, 1, 4)
    plt.plot(outliers, (np.array(objectives).astype(np.float) - np.array(bounds).astype(np.float)), 'go')
    plt.ylabel("(Obj - Bound)")

    plt.xlabel("Scene points")
    plt.draw()
    plt.pause(0.0001)

def save_data(config, outliers, times, objectives, bounds):
  filename_base = "tmp/solve_info_rotations%d_N%d_outliers%d_%s" % (
                                                     config["detector_options"]["rotation_constraint"],
                                                     config["detector_options"]["downsample_to_this_many_points"],
                                                     config["detector_options"]["add_this_many_outliers"],
                                                     datetime.datetime.now().strftime("%Y%m%d_%H:%M:%S"))
  plt.savefig(filename_base + ".png")
  dat = open(filename_base + ".dat", 'w')
  dat.write("outliers: %s\n" % list(outliers))
  dat.write("solve_times: %s\n" % times)
  dat.write("objectives: %s\n" % objectives)
  dat.write("bounds: %s\n" % bounds)


if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    save_data(miqp_config, outliers, solve_times, objectives, bounds)
    plt.close('all')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)
  plt.ion()

  # Make sure we have a tmp directory to spit data into.
  os.system("mkdir -p tmp")

  # Generate the sample point cloud using a pre-written config file
  os.system("run_point_cloud_generator config/point_cloud_generator_config.yaml tmp/cube_samples.pcd tmp/cube_sample_gt.yaml")

  miqp_config = get_miqp_mesh_model_detector_default_options()

  objectives = []
  bounds = []
  solve_times = []
  outliers = np.repeat([0, 5, 10, 15, 20, 25], 1)
  for i, add_this_many_outliers in enumerate(outliers):
    #try:
      miqp_config_file = "tmp/cube_miqp_mesh_model_config_outliers_%d.yaml" % add_this_many_outliers
      miqp_config["detector_options"]["add_this_many_outliers"] = int(add_this_many_outliers)
      #print miqp_config, miqp_config_file
      write_miqp_mesh_model_detector_config(miqp_config_file, miqp_config)

      output_file = "tmp/cube_miqp_mesh_model_fit_outliers%d.yaml" % add_this_many_outliers
      # Run the miqp detector using the sampled data we just generated
      os.system("run_miqp_mesh_model_detector tmp/cube_samples.pcd %s %s" % (miqp_config_file, output_file))

      # Parse ground truth and compare to the output
      gt_yaml = yaml.load(open("tmp/cube_sample_gt.yaml"))
      fit_yaml = yaml.load(open(output_file))


      objective = fit_yaml[0]["objective"]
      bound = fit_yaml[0]["bound"]
      solve_time = fit_yaml[0]["solve_time"]
      print objective

      objectives.append(objective)
      bounds.append(bound)
      solve_times.append(solve_time)

      update_plots(outliers[0:i+1], solve_times, objectives, bounds)

      print "N Scene points: ", list(outliers)
      print "Solve times: ", solve_times
      print "objectives, bounds: ", objectives, bounds
    #except Exception as e:
    #  print "Had a problem... ", e

  save_data(miqp_config, outliers, solve_times, objectives, bounds)
  plt.show(block=True)
  plt.close('all')