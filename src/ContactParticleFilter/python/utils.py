# os
import os

#spartan
import spartan.utils.utils as spartanUtils

def getCPFSourceDir():
    return os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'ContactParticleFilter')