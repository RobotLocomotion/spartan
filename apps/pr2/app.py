from director import robotsystem
from director import applogic
from PythonQt import QtGui, QtCore


# load a minimal robot system with ik planning
robotSystem = robotsystem.create(view, planningOnly=True)

# add the teleop and playback panels to the mainwindow
app.addWidgetToDock(robotSystem.teleopPanel.widget,
                        QtCore.Qt.RightDockWidgetArea)
app.addWidgetToDock(robotSystem.playbackPanel.widget,
                        QtCore.Qt.BottomDockWidgetArea).setVisible(False)

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
if ikPlanner.planningMode == 'pydrake':
    ikPlanner.plannerPub._setupLocalServer()


ikPlanner.robotNoFeet = True
ikPlanner.fixedBaseArm = False

# change the default animation mode of the playback panel
robotSystem.playbackPanel.animateOnExecute = True

# disable pointwise ik by default
ikPlanner.getIkOptions().setProperty('Use pointwise', False)
ikPlanner.getIkOptions().setProperty('Max joint degrees/s', 60)

# initialize the listener for the pose gui
ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)

# set the default camera view
applogic.resetCamera(viewDirection=[-1, 0, 0], view=view)
