from pymouse import PyMouse
m = PyMouse()
m.scroll(*m.position(), up=True, n=5)