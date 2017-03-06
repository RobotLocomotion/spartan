from director import mainwindowapp
from director import robotsystem
from director import applogic
from director import consoleapp

import contactfilter

# create a default mainwindow app
app = mainwindowapp.construct(globalsDict=globals())

# load a minimal robot system with ik planning
robotSystem = robotsystem.create(app.view, planningOnly=True)


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
