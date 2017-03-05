# Contact Particle Filter

Algorithm for localizing external contact using proprioceptive sensors.

# getting your environment setup

## you need either forcespro or gurobi installed.

forcespro: ensure that it is added to the PYTHONPATH so that 'import forcespro' works in python. Note it is installed on the robot-lab computer. For example on my machine I have

PYTHONPATH="/usr/local/bin/FORCES_PRO:${PYTHONPATH}"
export PYTHONPATH

gurobi: Again you need gurobi installed and it must be on the PYTHONPATH. So 'import gurobipy' needs to work in python. On my machine I have

export GUROBI_HOME="/home/manuelli/drc/software/externals/gurobi/gurobi562/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"


To choose a solver edit the solverType field in config/contact_particle_filter_config.yaml. It can be set to either gurobi or forcespro.


# Running the algorithm in simulation

1. source drake-distro/drake_environment.sh

2. Launch procman from drake-distro folder.
	- $ bot-procman-sheriff -l drake/examples/ContactParticleFilter/config/kuka_contact_particle_filter.pmd
3. Start entire ContactParticleFilter group in procman.
	- It's ok not to run the iiwa_status_to_residual translator when doing simulation, only needed for hardware.

4. Moving mouse over model in director, green arrow will appear. Left click to add a force. You should see the particles for the particle filter appear and estimate the contact location.

5. Force can be removed by deleting it in scene browser on left hand side of director. The forces are named things like 'iiwa_link_6 external force'

6. Particle Filter (PF) works best when force added to later links, i.e. 4,5,6,7.

7. If PF gets confused delete all forces and let it reset. Then add a force again. Alternatively restart PF from procman.

8. Launch bot-spy to see the lcm traffic being generated.

9. There are many settings in config/contact_particle_filter.yaml. I advise you not to adjust them until you know what they all do.


# Code Structure

- contactfilter.py is the main algorithm, it runs standalone in the procman process called 'contact-filter'

- 'drake-visualizer' in procman is essentially the kuka_ik_app with a few extra classes loaded. Namely
	- linkselection.py - does the green arrow stuff for adding forces.
	- externalforce.py - computes the true residual from the forces that were added
	- contactfiltervisualizer.py - draws CPF data that is transmitted over lcm.
	- 
