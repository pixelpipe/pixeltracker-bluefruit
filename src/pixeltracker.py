"""
pixeltracker class
"""

__name__ = "pixeltracker"
__version__ = '0.0.3'
__copyright__ = "Copyright ©2021 by pixelchain"
__description__ = "sensor tracker"

from time import sleep
from peripherals import Pixels, Buttons, Speaker, Accelerometer, Ble, Sensors, Activity, \
    Microphone, SimpleLogger
from gps import Gps
from utils import Timer, COLORS, _SEP

class Pixeltracker():
    """
    pixeltracker Class
    """
    def __init__(self):
        self.dataGetTime = Timer()
        self.testTime = Timer()
        self.clrErrTimer = Timer()
        self.msTimer = Timer()
        self.buttons = Buttons(self.AKeyPressed, self.BKeyPressed)
        self.pixels = Pixels()
        self.speaker = Speaker()
        self.gpsActivity = Activity(self.pixels, 0, COLORS["off"], COLORS["blue"], COLORS["green"],COLORS["orange"],COLORS["red"])
        self.bleActivity = Activity(self.pixels, 1, COLORS["off"], COLORS["blue"], COLORS["cyan"],COLORS["orange"],COLORS["red"])
        self.micActivity = Activity(self.pixels, 9, COLORS["off"], COLORS["blue"], COLORS["green"],COLORS["orange"],COLORS["red"])
        self.storageActivity = Activity(self.pixels, 4, COLORS["off"], COLORS["blue"], COLORS["green"],COLORS["orange"],COLORS["red"])
        # (self.itow2str(self.ITOW), self.LAT/10000000, self.LON/10000000, self.ALT/1000, self.HACC/1000000, self.VACC/1000000, self.BEARING, "" if self.DATA_ERROR == "" else "["+self.DATA_ERROR+"]")
        self.noGpsHeader = f"ACCX{_SEP}ACCY{_SEP}ACCZ{_SEP}TEMP{_SEP}LIGHT{_SEP}MAG{_SEP}LVL"
        self.gpsHeader = f"DAY{_SEP}TIME{_SEP}LAT{_SEP}LON{_SEP}ALT{_SEP}HACC{_SEP}VACC{_SEP}BEAR{_SEP}SIG{_SEP}"+ self.noGpsHeader
        self.logger = SimpleLogger(self.gpsHeader, 4, 1024 * 256, self.storageActivity) # 256 K x 4 files        
        self.gps = Gps(self.gpsDataHandler, self.gpsActivity)
        self.ble = Ble(self.bleActivity)
        self.mic = Microphone(self.micActivity)
        self.accelerometer = Accelerometer()
        self.sensors = Sensors()
        self._logSensorsOn = False
        self.writable = True

    def collectAndSendSensorData(self, inData):
        data = inData
        self.accelerometer.read()
        data += _SEP + self.accelerometer.toString()

        self.sensors.read()
        data += _SEP + self.sensors.toString()

        self.mic.read()
        data += _SEP + self.mic.toString()

        #self.dataGetTime.showAndMark("GPS Handler: ")
        if self.ble.isConnected:
            self.ble.write(data+"\n")

        self.logger.appendLine(data)

        return data


    def gpsDataHandler(self, gps, validData):

        gpsData = ""
        if validData:
            gpsData += gps.toString()
        else:
            print("{}".format(gps.DATA_ERROR))
            return
        
        data = self.collectAndSendSensorData(gpsData)
        print(data)

    def AKeyPressed(self):
        #self.gps.config()
        #return
        print("A")
        self.gps.toggle()
        if self.gps.isEnabled:
            self.logger.header = self.gpsHeader
            print(self.logger.header)
            self.speaker.playFile("on.wav")
        else:
            self.speaker.playFile("off.wav")

    def BKeyPressed(self):
        print("B")
        self.ble.toggle()
        if self.ble.isEnabled:
            self.speaker.playFile("on.wav")
        else:
            self.speaker.playFile("off.wav")        

    def __display(self, text):
        print(text)

    def sendSensorDataOnly(self):
        if not self.gps.isEnabled:
            if self.ble.isEnabled:
                self.collectAndSendSensorData("")

    def __mainLoop(self):
        self.clrErrTimer.stopWatch(5000)
        self.msTimer.stopWatch(0)
        print(self.noGpsHeader)
        while True:
            self.buttons.read()
            self.gps.read()
            self.mic.read()
            if self.msTimer.stopped():
                self.sendSensorDataOnly()
                self.msTimer.stopWatch(500)
            if self.clrErrTimer.stopped():
                self.pixels.clear()

    def run(self):
        """Runs the application"""
        self.__display("{} v{} {}".format(__name__,__version__,__description__))
        self.__display("{}".format(__copyright__))
        print("Running...")
        self.gps.enable()
        self.speaker.playFile("on.wav")        
        self.__mainLoop()