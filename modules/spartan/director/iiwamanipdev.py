#system
import os

# spartan
import spartan.manipulation.grasp_supervisor
import spartan.manipulation.background_subtraction
import spartan.calibration.handeyecalibration
import spartan.utils.utils as spartanUtils


def setupRLGDirector(globalsDict=None):
    graspSupervisor = spartan.manipulation.grasp_supervisor.GraspSupervisor.makeDefault()
    backgroundSubtraction = spartan.manipulation.background_subtraction.BackgroundSubtractionDataCapture.makeDefault()

    spartanSourceDir = spartanUtils.getSpartanSourceDir()
    handEyeCalibrationConfigFilename = os.path.join(spartanSourceDir, "src/catkin_projects/station_config/RLG_iiwa_1/hand_eye_calibration/cal.yaml")


    cal = spartan.calibration.handeyecalibration.HandEyeCalibration(globalsDict['robotSystem'], configFilename=handEyeCalibrationConfigFilename)
    cal.loadConfigFromFile()

    globalsDict['graspSupervisor'] = graspSupervisor
    globalsDict['backgroundSubtraction'] = backgroundSubtraction
    globalsDict['cal'] = cal