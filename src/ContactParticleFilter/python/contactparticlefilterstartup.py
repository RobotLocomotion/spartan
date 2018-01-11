__author__ = 'manuelli'

from director import transformUtils

import contactfilter
import contactfiltervisualizer
from contactfilterutils import EstRobotStatePublisher
import externalforce
import linkselection
import cpftester
import experimentmanager

import argparse
import director.applogic as app

import os

# spartan
from spartan.optitrack.optitrackvisualizer import OptitrackVisualizer


def startup(robotSystem, globalsDict=None):
    """"
    Creates classes relevant for the contact particle filter.
    """
    print "setting up the contact particle filter . . . "
    rs = robotSystem

    # setup the ROS node
    import rospy
    rospy.init_node('director_cpf')

    externalForce = externalforce.ExternalForce(rs)
    contactFilter = contactfilter.ContactFilter(rs.robotStateModel, rs.robotStateJointController)
    contactFilterVisualizer = contactfiltervisualizer.ContactFilterVisualizer(rs, rs.robotStateModel, refreshRate=5)
    linkSelection = linkselection.LinkWidget(rs.view, rs.robotStateModel, externalForce)
    linkSelection.start()

    estRobotStatePublisher = EstRobotStatePublisher(robotSystem)

    experimentManager = experimentmanager.ExperimentManager(rs, rs.robotStateJointController, linkSelection, externalForce, estRobotStatePublisher)


    optitrackVis = OptitrackVisualizer('OPTITRACK_FRAMES')

    # this aligs the kuka
    optitrackToWorld = transformUtils.transformFromPose([ 0.14486322,  0.21043359,  0.00027311], [ 0.49576806,  0.5083682 ,  0.50397359,  0.49171782])
    optitrackVis.optitrackToWorld = optitrackToWorld
    # optitrackVis.setTransformForIiwa2()
    optitrackVis.setEnabled(True)

    # kukaMarkersToKukaBase, transform from the markers called rlg_iiwa_2 to the base of the robot
    # this can be used in conjunction with pose of rlg_iiwa_2 to define optiTrackToWorld
    kukaMarkersToKukaBase = transformUtils.transformFromPose([-0.02094656, -0.00124157,  0.05628882], [ 0.49488455,  0.50835957,  0.50390494,  0.49268615])

    

    if globalsDict is not None:
        globalsDict['externalForce'] = externalForce
        globalsDict['contactFilter'] = contactFilter
        globalsDict['contactFilterVisualizer'] = contactFilterVisualizer
        globalsDict['linkSelection'] = linkSelection
        globalsDict['estRobotStatePublisher'] = estRobotStatePublisher
        globalsDict['experimentManager'] = experimentManager
        globalsDict['em'] = experimentManager
        globalsDict['optitrackVis'] = optitrackVis

        # globalsDict['cf'] = contactFilter
        # globalsDict['ef'] = externalForce

    print "contact particle filter initialized "


# launch script for testing
def testStartup(robotSystem, args, globalsDict=None):
    CPFTester = cpftester.CPFTester(globalsDict['externalForce'])
    CPFTester.loadContactPointDict(args.CPFFilename)
    CPFTester.args = args
    CPFTester.loadContactForceDataFromDict(args.CPFForceName)
    CPFTester.timer.start()

    # view = app.getDRCView()
    # view.hide()

    if globalsDict is not None:
        globalsDict['CPFTester'] = CPFTester
        globalsDict['linkSelection'].stop() # don't need this while running automated tests



