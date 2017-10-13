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
    self.task_queue = asynctaskqueue.AsyncTaskQueue()
    self.pending_tasks = []
    self.threads = []
    self.timer = TimerCallback(callback=self._on_timer)

  def _on_timer(self):
    
    # add all tasks in self.pending_tasks to the AsyncTaskQueue
    if self.pending_tasks:
      while True:
        try:
          self.task_queue.addTask(self.pending_tasks.pop(0))
        except IndexError:
          break

      # start the AsyncTaskQueue if it's not already running
      if self.task_queue.tasks and not self.task_queue.isRunning:
        self.task_queue.start()
    
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

  def call_on_main(self, func, *args, **kwargs):
    self.pending_tasks.append(lambda: func(*args, **kwargs))
    self.timer.start()

  def call_on_thread(self, func, *args, **kwargs):
    t = Thread(target=lambda: func(*args, **kwargs))
    self.threads.append(t)
    t.start()
    self.timer.targetFps = 1/self.interval
    self.timer.start()