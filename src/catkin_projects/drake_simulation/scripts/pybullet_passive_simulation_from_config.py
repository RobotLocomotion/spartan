#!/usr/bin/env python

# Given a configuration file (see this package's config folder for examples),
# simulates it.
# 
# Arguments:
#   config: Configuration file
#   rate: Desired real-time rate (will not run faster, might run slower)
# Press q to quit and r to restart the simulation.

import argparse
import os
import pybullet as p
import time
import yaml

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Configuration file to simulate.", type=str)
    parser.add_argument("-r", "--rate", help="Desired simulation rate (fraction of realtime)", type=float, default=1.0)
    parser.add_argument("-t", "--timestep", help="Simulation timestep", type=float, default=0.001)
    args = parser.parse_args()

    # Set up a simulation with a ground plane and desired timestep
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        
    keep_simulating = True
    while keep_simulating:
        p.resetSimulation()

        p.setGravity(0,0,-9.81)
        p.setTimeStep(args.timestep)

        # Read in configuration file
        config = yaml.load(open(args.config))

        print(config)

        if config["with_ground"] == True:
            p.loadURDF(os.environ["SPARTAN_SOURCE_DIR"] + "/build/bullet3/data/plane.urdf")

        # Models entry is a dictionary of model URDF strings
        model_dict = config["models"]

        # Add each model as requested
        drake_resource_root = os.environ["DRAKE_RESOURCE_ROOT"]
        ids = []
        for instance in config["instances"]:
            urdf = drake_resource_root + "/" + model_dict[instance["model"]]
            q0 = instance["q0"]
            position = q0[0:3]
            quaternion = p.getQuaternionFromEuler(q0[3:8])
            fixed = instance["fixed"]
            print("URDF ", urdf, " q0", q0)
            ids.append(p.loadURDF(urdf, position, quaternion, fixed))
            
        # Run simulation with time control
        start_time = time.time()
        sim_time = 0.0
        avg_sim_rate = -1.
        sim_rate_RC = 0.1 # RC time constant for estimating sim rate
        sim_rate_alpha = args.timestep / (sim_rate_RC + args.timestep) 
        last_print_time = time.time() - 100

        keep_going = True
        while keep_going:
            # Do a sim step
            start_step_time = time.time()
            p.stepSimulation()
            end_step_time = time.time()
            sim_time += args.timestep

            # Estimate sim rate
            this_sim_step_rate = args.timestep / (end_step_time - start_step_time)
            if avg_sim_rate < 0:
                # Initialization case
                avg_sim_rate = this_sim_step_rate
            else:
                avg_sim_rate = this_sim_step_rate * sim_rate_alpha + (1. - sim_rate_alpha) * avg_sim_rate

            # Sleep if we're running too fast
            elapsed = end_step_time - start_time
            target_sim_time = elapsed * args.rate
            if sim_time > target_sim_time:
                time.sleep(sim_time - target_sim_time)

            if time.time() - last_print_time > 0.1:
                last_print_time = time.time()
                print("Overall sim rate: ", sim_time / target_sim_time, ", current sim rate: ", avg_sim_rate, " at time ", sim_time)

            events = p.getKeyboardEvents()
            for key in events.keys():
                if events[key] & p.KEY_WAS_TRIGGERED:
                    if key == ord('q'):
                        print("Quitting")
                        keep_going = False
                        keep_simulating = False
                    elif key == ord('r'):
                        print("Restarting")
                        keep_going = False
                        break


