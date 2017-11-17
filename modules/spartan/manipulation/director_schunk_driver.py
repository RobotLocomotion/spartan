from spartan.manipulation.schunk_driver import SchunkDriver
from spartan.utils.taskrunner import TaskRunner



class DirectorSchunkDriver(object):

	def __init__(self, **kwargs):
		self.taskRunner = TaskRunner()
		self.taskRunner.callOnThread(self.initialize, **kwargs)

	def initialize(self, **kwargs):
		self.schunkDriver = SchunkDriver(**kwargs)

	def sendOpenGripperCommand(self):
		self.taskRunner.callOnThread(self.schunkDriver.sendOpenGripperCommand)

	def sendCloseGripperCommand(self):
		self.taskRunner.callOnThread(self.schunkDriver.sendCloseGripperCommand)

	def sendGripperCommand(self, position, force):
		self.taskRunner.callOnThread(self.schunkDriver.sendGripperCommand, posiition, force)