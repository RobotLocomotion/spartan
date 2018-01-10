# os
import os

#spartan
import spartan.utils.utils as spartanUtils

def getCPFSourceDir():
    return os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'ContactParticleFilter')

def getCPFDataDir():
    return os.path.join(getCPFSourceDir(), 'data', 'experiments')