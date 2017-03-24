from director import robotsystem
from director import applogic
from PythonQt import QtGui, QtCore


# load a minimal robot system with ik planning
robotSystem = robotsystem.create(view, planningOnly=True)

# add the teleop and playback panels to the mainwindow
app.addWidgetToDock(robotSystem.teleopPanel.widget,
                        QtCore.Qt.RightDockWidgetArea).setVisible(False)
app.addWidgetToDock(robotSystem.playbackPanel.widget,
                        QtCore.Qt.BottomDockWidgetArea).setVisible(False)

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
if ikPlanner.planningMode == 'pydrake':
    ikPlanner.plannerPub._setupLocalServer()

# todo: these should be set automaticaly from the config file
ikPlanner.robotNoFeet = True
ikPlanner.fixedBaseArm = False

# change the default animation mode of the playback panel
robotSystem.playbackPanel.animateOnExecute = True

# set some default ik options
ikPlanner.getIkOptions().setProperty('Use pointwise', False)
ikPlanner.getIkOptions().setProperty('Max joint degrees/s', 60)

# initialize the listener for the pose gui
ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)

# set the default camera view
applogic.resetCamera(viewDirection=[-1, 0, 0], view=view)

# initialize the bhpn translator
# this listens to JointConf lcm messages from BHPN and translates
# them into joint states for the PR2 model loaded in drake
import os
import sys
sys.path.append(os.path.dirname(__file__))
import bhpntranslator
bhpnTranslator = bhpntranslator.BHPNTranslator(robotSystem.robotStateJointController)
