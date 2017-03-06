__author__ = 'manuelli'

# this is startup script that gets executed on top of director_ik_app.py using --startup flag
"""
Script passed in with --startup option to directorPython that creates classes
relevant for the Contact Particle Filter

Usage
-------
used together with director_cpf_launch.py
"""

import os
import sys

# add relevant directories to path so Python imports can find them
SPARTAN_SOURCE_DIR = os.getenv('SPARTAN_SOURCE_DIR')
sys.path.append(os.path.join(SPARTAN_SOURCE_DIR, 'src/ContactParticleFilter/python'))


import contactparticlefilterstartup
contactparticlefilterstartup.startup(robotSystem, globals())

