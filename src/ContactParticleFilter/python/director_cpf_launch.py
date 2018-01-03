__author__ = 'manuelli'

"""
Launches a director instance with additional classes relevant to Contact Particle Filter.
Calls director with --startup option and file runContactFilterStartup.py
"""

import os
def main():

    spartan_source_dir = os.getenv('SPARTAN_SOURCE_DIR')

    if spartan_source_dir is None:
        raise EnvironmentError('The $SPARTAN_SOURCE_DIR variable must be set to the spartan source directory')
        return

    director_kuka_command = "directorPython $SPARTAN_SOURCE_DIR/drake/examples/kuka_iiwa_arm/director_ik_app.py" \
                            " --director_config $SPARTAN_SOURCE_DIR/drake/examples/kuka_iiwa_arm/director_config.json"
    startup_command = " --startup $SPARTAN_SOURCE_DIR/src/ContactParticleFilter/python/runContactParticleFilterStartup.py"


    shell_command = director_kuka_command + startup_command
    print shell_command
    os.system(shell_command)


if __name__ == '__main__':
    main()