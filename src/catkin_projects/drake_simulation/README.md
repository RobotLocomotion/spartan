Simulation Tools
=================
Provides tools for simulation in Spartan.

## Config files

This package provides configuration files describing
combinations of URDF files and their initial configurations,
as a way of specifying a simulation initial condition.
See `config/*` for examples. These are currently only
used by the passive simulation scripts in this directory, but
ought to eventually be piped into things like the IIWA simulation
(i.e. be able to support controlled robots too).

Each config file must list `models`, which correlate model
names (keys) to URDF files (values), with URDF filenames
being based from `DRAKE_RESOURCE_ROOT` (due to a limitation
in Drake's `FindResource`. 

Each config file must specify `with_ground`, i.e. whether the
sim should add its own ground plane.

Each config file must list `instances`, which correspond to
an instance of the specific model, at a given `q0`. Instances
may be `fixed` to indicate that they should not be dynamic.


## pybullet simulation

`scripts/pybullet_passive_simulation_from_config.py` takes
a configuration file, plus a timestep (-t) and/or sim rate (-r),
and simulates it in bullet. It'll pop up a view window and print
out the simulation rate. Invoke it directly or with `rosrun` --
it doesn't care about ROS or have many dependencies yet.

## drake simulation

`src/drake_passive_simulation_from_config.cc` links Drake to provide
a rigid body simulation. Run it with `--help` to see many arguments you
can provide.

## IIWA simulation (w/ Drake)

`src/iiwa_rlg_simulation/iiwa_rlg_simulation.cc` creates an IIWA simulation
using Drake and lots of support code (based on the KUKA IIWA simulation in
Drake examples). It is pretty hard-coded right now, but has some neat features:
- It simulates the ROS interface of the Schunk WSG50 gripper.
- It generates simulated RGBD images and publishes them on ROS topics. (It only
does images right now.)
- If you run it alongside a `kuka_plan_runner` and the IIWA manip app, you can
drive it. Launch the Director app first, then launch the sim, and you can use
the sim visualization to make the robot pick the object up.



