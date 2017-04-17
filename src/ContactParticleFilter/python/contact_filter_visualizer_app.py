from director import mainwindowapp
from director import robotsystem
from director import applogic
from director import consoleapp

import contactfilter


def loadCPFModules(robotSystem, globalsDict=None):
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

    print "contact particle filter initialized "



# create a default mainwindow app
app = mainwindowapp.construct(globalsDict=globals())

# load a minimal robot system with ik planning
robotSystem = robotsystem.create(app.view, planningOnly=True)
loadCPFModules(robotSystem, globals())
# set the default camera view
applogic.resetCamera(viewDirection=[-1,0,0], view=app.view)


# start the app
app.app.start()

