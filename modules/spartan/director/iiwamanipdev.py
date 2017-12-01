# spartan
import spartan.manipulation.grasp_supervisor
import spartan.manipulation.background_subtraction


def setupRLGDirector(globalsDict=None):
	graspSupervisor = spartan.manipulation.grasp_supervisor.GraspSupervisor.makeDefault()
	backgroundSubtraction = spartan.manipulation.background_subtraction.BackgroundSubtractionDataCapture.makeDefault()

	globalsDict['graspSupervisor'] = graspSupervisor
	globalsDict['backgroundSubtraction'] = backgroundSubtraction