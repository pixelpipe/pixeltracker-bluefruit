"""
pixeltracker class
"""

__name__ = "pixeltracker"
__version__ = '0.0.3'
__copyright__ = "Copyright ©2021 by pixelchain"
__description__ = "sensor tracker"

import gc
from time import sleep
from var import SEP, LOG_USING_UBX_FORMAT, UBXSEP
from peripherals import Pixels, Buttons, Speaker, Accelerometer, Ble, Sensors, Activity, \
    Microphone, SimpleLogger
from gps import Gps
from utils import Timer, COLORS

class Pixeltracker():
    """
    pixeltracker Class
    """
    def __init__(self):
        self.dataGetTime = Timer()
        self.memTimer = Timer()
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
        self.noGpsHeader = f"ACCX{SEP}ACCY{SEP}ACCZ{SEP}TEMP{SEP}LIGHT{SEP}MAG{SEP}LVL"
        self.gpsHeader = f"LAT{SEP}LON{SEP}ALT{SEP}SAT{SEP}YEAR{SEP}MONTH{SEP}TIME{SEP}HACC{SEP}VACC{SEP}SPEED{SEP}HEAD{SEP}HDACC{SEP}HDADV{SEP}SIGNAL{SEP}"+ self.noGpsHeader
        self.logger = SimpleLogger(self.gpsHeader, 4, 1024 * 256, self.storageActivity) # 256 K x 4 files        
        self.gps = Gps(self.gpsDataHandler, self.gpsActivity)
        self.ble = Ble(self.bleActivity)
        self.mic = Microphone(self.micActivity)
        self.accelerometer = Accelerometer()
        self.sensors = Sensors()
        self._logSensorsOn = False
        self.writable = True

    def collectAndSendSensorData(self, inData):
        #print(len(inData))
        data = inData
        self.accelerometer.read()        
        if LOG_USING_UBX_FORMAT:
            data += UBXSEP + self.accelerometer.ubx()
        else:
            data += SEP + self.accelerometer.toString()
        
        self.sensors.read()
        if LOG_USING_UBX_FORMAT:
            data += UBXSEP + self.sensors.ubx()
        else:            
            data += SEP + self.sensors.toString()

        self.mic.read()
        if LOG_USING_UBX_FORMAT:
            data += UBXSEP + self.mic.ubx()
        else:
            data += SEP + self.mic.toString()

        #self.dataGetTime.showAndMark("GPS Handler: ")
        if self.ble.isConnected:
            if LOG_USING_UBX_FORMAT:
                self.ble.write(data)                
            else:
                self.ble.write(data+"\n")

        if LOG_USING_UBX_FORMAT:
            self.logger.append(data)
        else:
            self.logger.appendLine(data)

        return data

    def gpsDataHandler(self, gps, validData):
        
        if validData:
            if LOG_USING_UBX_FORMAT:
                gpsData = gps.ubx()
            else:
                gpsData = gps.toString()
        else:
            print("{}".format(gps.DATA_ERROR))
            return
        
        data = self.collectAndSendSensorData(gpsData)
        
        print("{} {}".format(data,len(data)))

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
        self.clrErrTimer.stopWatch(10*1000)
        self.msTimer.stopWatch(0)
        #print(self.noGpsHeader)

        while True:

            self.buttons.read()

            self.gps.read()

            if not self.gps.isEnabled:
                self.mic.read()
                if self.msTimer.stopped():
                    self.sendSensorDataOnly()
                    self.msTimer.stopWatch(500)

            if self.clrErrTimer.stopped():
                self.pixels.clear()

            if self.memTimer.stopped():
                #print('Before gc free: {} allocated: {}'.format(gc.mem_free()/1024, gc.mem_alloc()/1024))
                gc.collect()
                #print('After gc free: {} allocated: {}'.format(gc.mem_free()/1024, gc.mem_alloc()/1024))
                #gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
                self.memTimer.stopWatch(60 * 1000)#15 * 60 * 1000) # 15min


    def run(self):
        """Runs the application"""
        self.__display("{} v{} {}".format(__name__,__version__,__description__))
        self.__display("{}".format(__copyright__))

        print("Running...")

        #self.gps.enable()
        self.speaker.playFile("on.wav")

        self.__mainLoop()