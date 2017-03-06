__author__ = 'manuelli'

import contactfilter
import contactfiltervisualizer
from contactfilterutils import EstRobotStatePublisher
import externalforce
import linkselection
import cpftester
import argparse
import director.applogic as app

import os


def startup(robotSystem, globalsDict=None):
    """"
    Creates classes relevant for the contact particle filter.
    """
    print "setting up the contact particle filter . . . "
    rs = robotSystem

    externalForce = externalforce.ExternalForce(rs)
    contactFilter = contactfilter.ContactFilter(rs.robotStateModel, rs.robotStateJointController)
    contactFilterVisualizer = contactfiltervisualizer.ContactFilterVisualizer(rs, rs.robotStateModel)
    linkSelection = linkselection.LinkWidget(rs.view, rs.robotStateModel, externalForce)
    linkSelection.start()

    estRobotStatePublisher = EstRobotStatePublisher(robotSystem)

    if globalsDict is not None:
        globalsDict['externalForce'] = externalForce
        globalsDict['contactFilter'] = contactFilter
        globalsDict['contactFilterVisualizer'] = contactFilterVisualizer
        globalsDict['linkSelection'] = linkSelection
        globalsDict['estRobotStatePublisher'] = estRobotStatePublisher

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



