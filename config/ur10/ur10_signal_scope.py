numJoints = 6
jointNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

addPlot(timeWindow=15, yLimits=[-3.14, 3.14])
addSignals('EST_ROBOT_STATE', msg.utime, msg.joint_position, jointNames, keyLookup=msg.joint_name)
addSignals('ROBOT_COMMAND', msg.utime, msg.joint_position, range(numJoints))
