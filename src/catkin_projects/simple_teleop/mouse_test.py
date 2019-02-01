from pymouse import PyMouseEvent

def fibo():
    a = 0
    yield a
    b = 1
    yield b
    while True:
        a, b = b, a+b
        yield b

class Clickonacci(PyMouseEvent):
    def __init__(self):
        PyMouseEvent.__init__(self)
        self.fibo = fibo()

    def click(self, x, y, button, press):
        '''Print Fibonacci numbers when the left click is pressed.'''

        print x, y, button, press

        if button == 1:
            if press:
                print(self.fibo.next())
        #else:  # Exit if any other mouse button used
        #    self.stop()

    def move(self, x, y):
        print "the mouse was moved to", x, y
        return x, y

    def scroll(self, x, y, direction):
        print "the mouse scrolled"
        return x, y, direction

C = Clickonacci()
C.run()