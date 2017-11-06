# spartan
import spartan.manipulation.grasp_supervisor


def setupRLGDirector(globalsDict=None):
	graspSupervisor = spartan.manipulation.grasp_supervisor.GraspSupervisor.makeDefault()

	globalsDict['graspSupervisor'] = graspSupervisor