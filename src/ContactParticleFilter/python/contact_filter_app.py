from director import mainwindowapp
from director import robotsystem
from director import applogic
from director import consoleapp

import contactfilter


def makeRobotSystem(view):
    factory = robotsystem.RobotSystemFactory()
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=False,
                                         useTeleop=True)
    return factory.construct(view=view, options=options)


# create a default mainwindow app
app = mainwindowapp.MainWindowAppFactory().construct(globalsDict=globals())
mainwindowapp.MainWindowPanelFactory().construct(app=app.app, view=app.view)

# load a minimal robot system with ik planning
robotSystem = makeRobotSystem(app.view)


# set the default camera view
applogic.resetCamera(viewDirection=[-1,0,0], view=app.view)

contactFilter = contactfilter.ContactFilter(robotSystem.robotStateModel, robotSystem.robotStateJointController)
contactFilter.start()

# toggle show/hide of window
showWindow = False
if showWindow:
    app.app.start()
else:
    consoleapp.ConsoleApp.start()
