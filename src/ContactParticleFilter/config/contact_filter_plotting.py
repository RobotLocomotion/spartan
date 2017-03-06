

jointIdx = range(1,8)
armJoints = []
for i in jointIdx:
	jointName = 'iiwa_joint_' + str(i)
	armJoints.append(jointName)


baseJoints = []
for a in ['x','y','z','roll','pitch','yaw']:
	baseJointName = 'base_' + a
	baseJoints.append(baseJointName)

jointNames = armJoints
jointIdx = range(7)
timeWindow = 10
addPlot(timeWindow=timeWindow, yLimits=[-10,10])

addSignals('RESIDUAL_OBSERVER_STATE', msg.utime, msg.residual, jointNames, keyLookup=msg.joint_name)
# addSignals('RESIDUAL_ACTUAL', msg.utime, msg.residual, jointNames, keyLookup=msg.joint_name)

addPlot(timeWindow=timeWindow, yLimits=[-10,10])

# logLikelihood is actually the squared error
addSignal('CONTACT_FILTER_POINT_ESTIMATE', msg.utime, msg.logLikelihood)

addPlot(timeWindow=timeWindow, yLimits=[-1,5])

# logLikelihood is actually the squared error
addSignal('CONTACT_FILTER_POINT_ESTIMATE', msg.utime, msg.num_contact_points)
