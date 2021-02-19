import time

_SEP = ","

# Basic Color Table
COLORS = {
    "red"   : (255, 0, 0),
    "green" : (0, 255, 0),
    "cyan" : (0, 255, 255),
    "yellow" : (255, 255, 0),
    "blue" : (0, 0, 255),
    "white" : (255, 255, 255),
    "pink" : (255, 0, 100),
    "orange" : (230, 80, 0),
    "off" : (0,0,0)
}

class Timer():
    """Timer class"""
    def __init__(self):
        self._NOW = 0
        self._PREV = self._NOW
        self.mark()
        self.DELTA = 0
        self._stopped = False

    def mark(self):
        self._NOW = time.monotonic_ns()
        self._PREV = self._NOW

    def stopWatch(self, msTime):
        self._stopWatch = msTime
        self._stopped = False
        self.mark()

    def stopped(self):
        if self._stopped:
            return True
        self._NOW = time.monotonic_ns()
        self.calc()
        if self._DELTA > (self._stopWatch * 1000000):
            self.stooped = True
            return True
        return False

    def calc(self):
        self._DELTA = self._NOW - self._PREV

    def showAndMark(self, msg = ""):
        self._NOW = time.monotonic_ns()
        self.calc()
        self.mark()
        self.print(msg)

    def show(self, msg = ""):
        self._NOW = time.monotonic_ns()
        self.calc()
        self.print(msg)

    def print(self, msg = ""):
        print("{}{:.2f} ms".format(" " + msg if msg != "" else "", self._DELTA/1000/1000))
