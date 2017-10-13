import sys
import time
import sys
from threading import Thread

from director import asynctaskqueue
from director.timercallback import TimerCallback


class TaskRunner(object):

  def __init__(self):
    self.interval = 1/60.0
    sys.setcheckinterval(1000)
    #sys.setswitchinterval(self.interval)			# sys.setswitchinterval is only Python 3
    self.taskQueue = asynctaskqueue.AsyncTaskQueue()
    self.pendingTasks = []
    self.threads = []
    self.timer = TimerCallback(callback=self._onTimer)

  def _onTimer(self):
    # Give up control to another python thread in self.threads
    # that might be running
    time.sleep(self.interval)
    
    # add all tasks in self.pendingTasks to the AsyncTaskQueue
    if self.pendingTasks:
      while True:
        try:
          self.taskQueue.addTask(self.pendingTasks.pop(0))
        except IndexError:
          break

      # start the AsyncTaskQueue if it's not already running
      if self.taskQueue.tasks and not self.taskQueue.isRunning:
        self.taskQueue.start()
    
    # check which threads are live
    liveThreads = []
    for t in self.threads:
      if t.is_alive():
        liveThreads.append(t)

    # only retain the live threads
    self.threads = liveThreads

    # if no liveThreads then stop the timer
    if len(self.threads) == 0:
      self.timer.stop()

  def callOnMain(self, func, *args, **kwargs):
    self.pendingTasks.append(lambda: func(*args, **kwargs))
    self.timer.start()

  def callOnThread(self, func, *args, **kwargs):
    t = Thread(target=lambda: func(*args, **kwargs))
    self.threads.append(t)
    t.start()
    self.timer.targetFps = 1/self.interval
    self.timer.start()