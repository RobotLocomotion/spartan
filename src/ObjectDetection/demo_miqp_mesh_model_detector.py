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
      # options of rot constraints:
      # 0: constrained to ground truth
      # 1: unconstrained
      # 2: columnwise and row-wise L1-norm >= 1
      # 3: McCormick quaternion
      # 4: McCormick directly on rotmat
      # 5: Conservative RPY limits
      rotation_constraint = 4,
      rotation_constraint_num_faces = 2,
      
      allow_outliers = False,
      phi_max = 0.01,

      use_initial_guess = True,
      corruption_amount = 0.0,

      downsample_to_this_many_points = 15,

      gurobi_int_options = dict(
        Threads = 10
      ),

      gurobi_float_options = dict(
        TimeLimit = 1200,
        FeasRelaxBigM = 100,
        MIPGap = 0.00
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
def update_plots(corruption_amounts, times, objectives):
    plt.subplot(2, 1, 1)
    plt.plot(corruption_amounts, times, 'ro')
    plt.ylabel("Solve Times")

    plt.subplot(2, 1, 2)
    plt.plot(corruption_amounts, objectives, 'go')
    plt.ylabel("Solve Objective")

    plt.xlabel("Problem Size")
    plt.draw()
    plt.pause(0.0001)

def save_data(config, corruption_amounts, times, objectives):
  filename_base = "solve_info_rotations%d_N%d_%s" % (config["detector_options"]["rotation_constraint"],
                                                     config["detector_options"]["downsample_to_this_many_points"],
                                                     datetime.datetime.now().strftime("%Y%m%d_%H:%M:%S"))
  plt.savefig(filename_base + ".png")
  dat = open(filename_base + ".dat", 'w')
  dat.write("corruption_amounts: %s\n" % list(corruption_amounts))
  dat.write("solve_times: %s\n" % times)
  dat.write("objectives: %s\n" % objectives)


if __name__ == "__main__":
   
  def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    save_data(miqp_config, corruption_amounts, solve_times, scores)
    plt.close('all')
    exit(0)

  signal.signal(signal.SIGINT, signal_handler)
  plt.ion()

  # Make sure we have a tmp directory to spit data into.
  os.system("mkdir -p tmp")

  # Generate the sample point cloud using a pre-written config file
  os.system("run_point_cloud_generator config/point_cloud_generator_config.yaml tmp/cube_samples.pcd tmp/cube_sample_gt.yaml")

  miqp_config = get_miqp_mesh_model_detector_default_options()

  scores = []
  solve_times = []
  corruption_amounts = np.repeat(np.linspace(0.0, 0.1, 11), 10)
  for i, corruption_amount in enumerate(corruption_amounts):
    miqp_config_file = "tmp/cube_miqp_mesh_model_config_corrupt_%1.3f.yaml" % corruption_amount
    miqp_config["detector_options"]["corruption_amount"] = float(corruption_amount)
    write_miqp_mesh_model_detector_config(miqp_config_file, miqp_config)

    # Run the bogo detector using the sampled data we just generated
    os.system("run_miqp_mesh_model_detector tmp/cube_samples.pcd %s tmp/cube_miqp_mesh_model_fit.yaml" % miqp_config_file)

    # Parse ground truth and compare to the output
    gt_yaml = yaml.load(open("tmp/cube_sample_gt.yaml"))
    fit_yaml = yaml.load(open("tmp/cube_miqp_mesh_model_fit.yaml"))

    score = fit_yaml[0]["score"]
    solve_time = fit_yaml[0]["solve_time"]

    scores.append(score)
    solve_times.append(solve_time)

    update_plots(corruption_amounts[0:i+1], solve_times, scores)

    print "Corruptions: ", list(corruption_amounts)
    print "Solve times: ", solve_times
    print "Scores: ", scores

  save_data(miqp_config, corruption_amounts, solve_times, scores)
  plt.show(block=True)
  plt.close('all')