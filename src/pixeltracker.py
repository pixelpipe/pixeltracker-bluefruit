"""
pixeltracker class
"""

__name__ = "pixeltracker"
__version__ = '0.0.3'
__copyright__ = "Copyright ©2021 by pixelchain"
__description__ = "sensor tracker"

from time import sleep
from peripherals import Pixels, Buttons, Speaker, Accelerometer, Ble, Gps, Sensors, Activity, \
    Microphone, Timer, SimpleLogger, COLORS

class Pixeltracker():
    """
    pixeltracker Class
    """
    def __init__(self):
        self.dataGetTime = Timer()
        self.testTime = Timer()
        self.clrErrTimer = Timer()
        self.buttons = Buttons(self.AKeyPressed,self.BKeyPressed)
        self.pixels = Pixels()
        self.speaker = Speaker()
        self.gpsActivity = Activity(self.pixels, 0, COLORS["off"], COLORS["blue"], COLORS["green"],COLORS["orange"],COLORS["red"])
        self.bleActivity = Activity(self.pixels, 1, COLORS["off"], COLORS["blue"], COLORS["cyan"],COLORS["orange"],COLORS["red"])
        self.micActivity = Activity(self.pixels, 9, COLORS["off"], COLORS["blue"], COLORS["green"],COLORS["orange"],COLORS["red"])
        self.storageActivity = Activity(self.pixels, 4, COLORS["off"], COLORS["blue"], COLORS["green"],COLORS["orange"],COLORS["red"])
        self.logger = SimpleLogger(10, 128 * 1024, self.storageActivity) # 128 K x 10 files
        self.gps = Gps(self.gpsDataHandler, self.gpsActivity)
        self.ble = Ble(self.bleActivity)
        self.mic = Microphone(self.micActivity)
        self.accelerometer = Accelerometer()
        self.sensors = Sensors()
        self.writable = True

    def gpsDataHandler(self, gps, validData):

        gpsData = ""
        if validData:
            gpsData += gps.toString()
        else:
            print("{}".format(gps.DATA_ERROR))
            return

        self.accelerometer.read()
        gpsData += "\t" + self.accelerometer.toString()

        self.sensors.read()
        gpsData += "\t" + self.sensors.toString()

        self.mic.read()
        gpsData += "\t" + self.mic.toString()

        #self.dataGetTime.showAndMark("GPS Handler: ")
        if self.ble.isConnected:
            self.ble.write(gpsData)

        self.logger.appendLine(gpsData)

        print(gpsData)

    def AKeyPressed(self):
        print("A")
        self.gps.toggle()

    def BKeyPressed(self):
        print("B")
        self.ble.toggle()

    def __display(self, text):
        print(text)

    def __mainLoop(self):
        self.clrErrTimer.stopWatch(5000)
        while True:
            self.buttons.read()
            self.gps.read()
            self.mic.read()
            if self.clrErrTimer.stopped():
                self.pixels.clear()

    def run(self):
        """Runs the application"""
        self.__display("{} v{} {}".format(__name__,__version__,__description__))
        self.__display("{}".format(__copyright__))
        print("Running...")
        self.speaker.playFile("on.wav")
        self.__mainLoop()