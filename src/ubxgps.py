import time
import board
import neopixel
import struct
import busio
import digitalio
import binascii
import utils

FLAG_LOG_ON = True

GPS_LED     = 0
SEN_LED     = 1
ACC_LED     = 2
MIC_LED     = 3
DSK_LED     = 4
BLE_LED     = 5
CON_LED     = 9

class UbxGps():

    def __init__(self, onDataHandler, bufferSize):
        self._onDataHandler = onDataHandler
        self._buffer = bytearray(bufferSize)
        self._ubx = b''
        self._uart = busio.UART(board.TX, board.RX, baudrate=9600)
        self._enabled = digitalio.DigitalInOut(board.D9)
        self._enabled.direction = digitalio.Direction.OUTPUT
        self.__ubxLeft = b""
        self.__ubxNext = b""

    def read(self):
        """Read Buffer from UART"""

        if not self._enabled:
            return
    
        self._uart.readinto(self._buffer)

        magicIndex = self._buffer.find(b'\xb5\x62')        
        if magicIndex>=0:            
            # move leftover into ubx and process it            
            self._ubx+=self._buffer[:magicIndex]
            if self._onDataHandler:
                self._onDataHandler()
            # init ubx with the next bytes
            self._ubx=self._buffer[magicIndex:]
        else:
            # append buffer to ubx
            self._ubx += bytes(self._buffer)

        # ToDo: the only problem is when 1st char of the buffer is 0xb5
        #       or last one is 0x62

    def toHex(self):
        """Return human readable data"""
        return binascii.hexlify(self._ubx)

    def ubxBuffer(self):
        """Return current UBX Buffer"""
        return self._buffer

    def toUbx(self):
        """Return the current UBX"""
        return self._ubx

    def toJson(self):
        str = ""
        ubxMagic1 = self._ubx[0] 
        ubxMagic2 = self._ubx[1]
        ubxClass = self._ubx[2]
        ubxId = self._ubx[3]
        ubxLen = self._ubx[4] + self._ubx[5] * 256
        packetLen = 6 + ubxLen + 2        
        if len(self._ubx) != packetLen:
            print("Broken UBX: Expected {}  got {}".format(packetLen, len(self._ubx)))
            print(binascii.hexlify(self._ubx))
            json = {
                "type":"ubxNavPvt"
            }
            str = "{}".format(json)
            return str
        if ubxClass == 0x01 or ubxId == 0x07:
            x = struct.unpack_from('<LHBBBBBBLLBBBBllllLLlllllLLHBBBBBBlhL', self._ubx, 6)
            json = {
                "type":"ubxNavPvt",
                "itow":x[0],
                "year":x[1],"month":x[2],"day":x[3],"hour":x[4],"min":x[5],"sec":x[6],
                "valid":x[7],"tacc":x[8],"nano":x[9],"fixtype":x[10],"flags":x[11],"flags2":x[12],
                "numsv":x[13],"lon":x[14],"lat":x[15],"alt":x[16],"hmsl":x[17],"hacc":x[18],"vacc":x[19],
                "veln":x[20],"vele":x[21],"veld":x[22],"speed":x[23],
                "headmot":x[24],"sacc":x[25],"headacc":x[26],"pdop":x[27],"flags3":x[28],"rsvd1":x[29],"headveh":x[30],
                "magdec":x[31],"magacc":x[22]
                }
            str = "{}".format(json)
        return str


    def enable(self):        
        self._enabled.value = True
        print("GPS ON" if self._enabled.value else "GPS OFF")

    def disable(self):
        self._enabled.value = False

    @property
    def enabled(self):
        return self._enabled.value

class Leds():
    def __init__(self, pixelCount, brightness):
        self._pixels = neopixel.NeoPixel(board.NEOPIXEL, pixelCount, brightness=brightness, auto_write=False)
        self.toggles = [False] * pixelCount
        self._pixelCount = pixelCount
        self._brightness = brightness
        self.allBlink()

    def allBlink(self):
        self._pixels.fill(255)
        self._pixels.show()
        self._pixels.fill(0)
        self._pixels.show()

    def allOn(self):
        self._pixels.fill(255)
        self._pixels.show()

    def allOff(self):
        self._pixels.fill(0)
        self._pixels.show()

    def set(self, id, color):
        self._pixels[id] = color
        self._pixels.show()

    def blink(self, id, color):
        self.set(id, color)
        self.clear(id)

    def clear(self, id):
        self._pixels[id] = (0,0,0)
        self._pixels.show()

    def togle(self, id, color):
        toggles[id] = not toggles[id]
        if toggles[id]:
            self._pixels[id] = color
        else:
            self._pixels[id] = (0,0,0)
        self._pixels.show()

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

class Ble():
    def __init__(self, onConnectionStateChanged, onAdvertising = None, onWrite = None):
        self._ble = BLERadio()
        self._uart_server = UARTService()
        self._onAdvertising = onAdvertising
        self._onWrite = onWrite
        self._advertisement = ProvideServicesAdvertisement(self._uart_server)
        self._isAdvertising = False
        self._onAdvertising = onAdvertising
        self._onWrite = onWrite
        self.__oldConnectionState = False
        self._onConnectionStateChanged = onConnectionStateChanged

    def write(self, data):
        if self._ble.connected:
            self._uart_server.write(data)
            if self._onWrite:
                self._onWrite(data)

    def startAdvertising(self):
        if not self._ble.connected:
            self.stopAdvertising()
        self._ble.start_advertising(self._advertisement)
        print("Start Phone App and connect")
        print("Started Advertising to BLE devices")
        if self._onAdvertising:
            self._onAdvertising()
        self._isAdvertising = True

    def stopAdvertising(self):
        #if self._ble.connected:
        print("Stopped Advertising")
        self._ble.stop_advertising()
        self._isAdvertising = False
    
    def read(self):

        if self._ble.connected != self.__oldConnectionState:
            self._onConnectionStateChanged()

        self.__oldConnectionState = self._ble.connected

    @property
    def isAdvertising(self):
        return self._isAdvertising

    @property
    def connected(self):
        return self._ble.connected

import digitalio
try:
    from audiocore import WaveFile
except ImportError:
    from audioio import WaveFile

try:
    from audioio import AudioOut
except ImportError:
    try:
        from audiopwmio import PWMAudioOut as AudioOut
    except ImportError:
        pass  # not always supported by every board!

class Speaker():
    """Handle the built-in speaker and makes noise"""
    def __init__(self):
        # Enable the speaker
        self._speakerEnable = digitalio.DigitalInOut(board.SPEAKER_ENABLE)
        self._speakerEnable.direction = digitalio.Direction.OUTPUT
        self._speakerEnable.value = True

    def playFile(self, filename):
        wave_file = open(filename, 'rb')
        with WaveFile(wave_file) as wave:
            with AudioOut(board.SPEAKER) as audio:
                audio.play(wave)
                while audio.playing:
                    pass

class Buttons():
    """Handles the button states"""
    def __init__(self, keyHandlerA, keyHandlerB):
        self.__buttonAState = False
        self.__buttonBState = False
        self._onButtonAPressed = keyHandlerA
        self._onButtonBPressed = keyHandlerB
        # Configure the A Button
        self._buttonA = digitalio.DigitalInOut(board.BUTTON_A)
        self._buttonA.switch_to_input(pull=digitalio.Pull.DOWN)
        # Configure the B Button
        self._buttonB = digitalio.DigitalInOut(board.BUTTON_B)
        self._buttonB.switch_to_input(pull=digitalio.Pull.DOWN)

    def read(self):
        if self._buttonA.value:
            if not self._buttonAState:
                self._onButtonAPressed()
                self._buttonAState = True
        else:
            self._buttonAState = False

        if self._buttonB.value:            
            if not self._buttonBState:
                self._onButtonBPressed()
                self._buttonBState = True
        else:
            self._buttonBState = False

    @property
    def buttonA(self):
        return self._buttonA

    @property
    def buttonB(self):
        return self._buttonB

import storage
class SimpleLogger:
    """Basic logging on the local storage"""
    def __init__(self, onLogData, header, logs, logSize):
        self._onLogData = onLogData
        self._logFile = "0.txt"
        self._logs = logs
        self._crtLog = 0
        self._logSize = logSize
        self._writable = True
        self._loggedBytes = 0
        self._header = header
        self._diskState = "Ok"

    def checkNextFile(self):
        # check if log file size exceeds the allowed max size
        if self._loggedBytes > self._logSize:
            # increment log number
            self._crtLog = self._crtLog + 1
            # circular logs
            if self._crtLog >= self._logs:
                self._crtLog = 0
            # build the filename
            self._logFile = "{}.txt".format(self._crtLog)
            # initialize the new file and write the header befor logging
            self.writeData(self._header, 'w')
            """
            if self._writable:
                with open(self._logFile, 'w') as fp:
                    fp.write(self._header)
                    fp.flush()
                    self._loggedBytes = len(self._header)
            else:
                print("{} {} [{}/{}]".format(self._diskState, self._logFile, len(self._header), self._loggedBytes))
                #print(self._header)
                self._loggedBytes = len(self._header)
            """

    def writeData(self, data, flag):
        # write data
        if self._writable:  
            with open(self._logFile, flag) as fp:
                fp.write(data)
                fp.flush()
                self._loggedBytes = self._loggedBytes + len(data)
        else:
            print("{} {} [{}/{}]".format(self._diskState, self._logFile, len(data), self._loggedBytes))
            #print(data)
            self._loggedBytes = self._loggedBytes + len(data)

        if self._onLogData:
            self._onLogData(data, self._diskState)
    
    def append(self, data):
        """Appends data to a file"""
        try:
            # logs only if writable, otherwise sends to stdio
            self.checkNextFile()
            self.writeData(data, 'a')
        except OSError as e:
            # cant write -> not writable
            if e.args[0] == 28:  # If the file system is full...
                self._diskState = "Disk full"
            else:
                self._diskState = "Disk protected"
            self._writable = False

    def appendLine(self, data):
        """Appends a line of data to a file"""
        self.append(data+"\n")

    @property
    def canWrite(self):
        return self._writable

class Timer():
    """Timer class"""
    def __init__(self, enabled = True):
        self._NOW = 0
        self._PREV = self._NOW
        self.mark()
        self._DELTA = 0
        self._stopped = True
        self._stopWatch = 0
        self._enabled = enabled

    def mark(self):
        self._NOW = time.monotonic_ns()
        self._PREV = self._NOW

    def stopWatch(self, msTime):
        self._stopWatch = msTime
        self._stopped = False
        self.mark()

    def reset(self):
        self._stopped = False
        self.mark()        

    def stop(self):
        self._stopped = True

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
        print("{}{:.2f} ms".format(msg if msg != "" else "", self._DELTA/1000/1000))


    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    @property
    def enabled(self):
        return self._enabled

import busio
import adafruit_lis3dh
class Accelerometer():
    """"""
    def __init__(self):
        if hasattr(board, "ACCELEROMETER_SCL"):
            self._acc_i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
            self._lis3dh = adafruit_lis3dh.LIS3DH_I2C(self._acc_i2c, address=0x19)
        else:
            self._acc_i2c = busio.I2C(board.SCL, board.SDA)
            self._lis3dh = adafruit_lis3dh.LIS3DH_I2C(self._acc_i2c)
        # Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
        self._lis3dh.range = adafruit_lis3dh.RANGE_8_G
        self._lis3dh.set_tap(2,60)
        self._enabled = False
        self._data = [0,0,0]

    def read(self):

        if not self._enabled:
            return            

        #self._data = [ value / adafruit_lis3dh.STANDARD_GRAVITY for value in self._lis3dh.acceleration]
        #for i in range(3):
        #    self._data[i] = self._lis3dh.acceleration[i]
        self._data = self._lis3dh.acceleration

    def toUbx(self):
        return struct.pack("<BBHfff",ord('S'), ord('A'), 4, self._data[0], self._data[1],self._data[2])

    def toJson(self):
        json = {
            'accx' : self._data[0],
            'accy' : self._data[1],
            'accz' : self._data[2]
        }        
        return "{}".format(json)

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    @property
    def enabled(self):
        return self._enabled

import audiobusio
import array
import math
class Microphone():
    """ Samples data from the microphone and returns the noise
        magnitude and level"""

    def __init__(self, numberOfSamples = 64):
        self._NUM_SAMPLES = numberOfSamples
        self._SCALE_EXPONENT = math.pow(10, 2 * -0.1) # curve is 2
        self._mic = audiobusio.PDMIn(board.MICROPHONE_CLOCK, board.MICROPHONE_DATA,sample_rate=16000, bit_depth=16)
        # Record an initial sample to calibrate. Assume it's quiet when we start.
        self._samples = array.array('H', [0] * self._NUM_SAMPLES)
        self._mic.record(self._samples, len(self._samples))
        # Set lowest level to expect, plus a little.
        self._input_floor = self.normalized_rms(self._samples) + 10
        self._input_ceiling = self._input_floor + 500
        self._level = 0
        self._magnitude = 0

    def constrain(self, value, floor, ceiling):
        """Restricts value to be between floor and ceiling."""
        return max(floor, min(value, ceiling))

    def log_scale(self, input_value, input_min, input_max, output_min, output_max):
        """Scale input_value between output_min and output_max, exponentially."""
        normalized_input_value = (input_value - input_min) / (input_max - input_min)
        return output_min + math.pow(normalized_input_value, self._SCALE_EXPONENT) * (output_max - output_min)

    def normalized_rms(self, values):
        """Remove DC bias before computing RMS."""
        minbuf = int(self.mean(values))
        samples_sum = sum(float(sample - minbuf) * (sample - minbuf) for sample in values)
        return math.sqrt(samples_sum / len(values))

    def mean(self, values):
        """Calculates the mean value"""
        return sum(values) / len(values)

    def read(self):
        """read samples data from microphone"""

        # Record samples
        self._mic.record(self._samples, len(self._samples))

        # Calculates the magnitude
        self._magnitude = self.normalized_rms(self._samples)

        # Compute scaled logarithmic reading in the range 0 to NUM_PIXELS
        self._level = self.log_scale(self.constrain(self._magnitude, self._input_floor, self._input_ceiling),self._input_floor, self._input_ceiling, 0, 255)

    def toJson(self):
        json = {
            "magnitude" : self._magnitude
            , "level" : self._level
            #, "samples" : self._samples
        }
        return "{}".format(json)

    def toUbx(self):
        return struct.pack("<BBHff",ord('S'), ord('M'), 8,self._magnitude/100, self._level/100)

import analogio
import adafruit_thermistor
class Sensors():
    """Sensors Class"""
    def __init__(self):
        # Configure the Thermistor for reading the temperature
        self._thermistor = adafruit_thermistor.Thermistor(board.TEMPERATURE, 10000, 10000, 25, 3950)
        self._light = analogio.AnalogIn(board.LIGHT)
        self._enabled = False
        self._data = [0,0,0]

    def scale(self, value):
        """Scale the light sensor values from 0-65535 (AnalogIn range)
        to 0-50 (arbitrarily chosen to plot well with temperature)"""
        return  (value / 65535 * 50) * 10

    def read(self):

        if not self._enabled:
            return

        self._data[0] = self._thermistor.temperature
        self._data[1] = self.scale(self._light.value)

    def toUbx(self):
        return struct.pack("<BBHff",ord('S'), ord('S'), 4, self._data[0], self._data[1])

    def toJson(self):
        json = {
            'temp' : self._data[0],
            'light' : self._data[1]
        }
        return "{}".format(json)

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    @property
    def enabled(self):
        return self._enabled

    @property
    def thermistor(self):
        return self._thermistor

    @property
    def light(self):
        return self._light

class PixelTrackerLite:
    """Main Loop"""
    def __init__(self):
        self._gps = UbxGps(self.sendAllData, 50)
        self._leds = Leds(10, 0.05)
        self._ble = Ble(self.onBleConnectionStateChanged, self.onBleAdvertising, self.onWrite)
        self._speaker = Speaker()
        self._buttons = Buttons(self.AKeyPressed, self.BKeyPressed)
        self._sensors = Sensors()
        self._accelerometer = Accelerometer()
        self._microphone = Microphone(32)
        self._logger = SimpleLogger(self.onLogData, b'UBX', 5, 250 * 1000 + 3)
        self.perf = Timer()

    def sendAllData(self):
        """Send All Data"""
        self.perf.showAndMark("SendAllData : ")
        self._leds.set(GPS_LED, (255, 0, 0))
        self._leds.clear(GPS_LED)
        self._logger.append(self._gps.toUbx())

        self._leds.set(SEN_LED, (0, 155, 0))
        self._leds.clear(SEN_LED)
        self._sensors.read()
        self._logger.append(self._sensors.toUbx())

        self._leds.set(ACC_LED, (0, 155, 0))
        self._leds.clear(ACC_LED)
        self._accelerometer.read()
        self._logger.append(self._accelerometer.toUbx())
        
        self._leds.set(MIC_LED, (0, self._microphone._level, 0))
        self._leds.clear(MIC_LED)
        self._logger.append(self._microphone.toUbx())

        if self._ble.connected:            
            self._ble.write(self._gps.toJson())
            self._ble.write('\n')
            self._ble.write(self._sensors.toJson())
            self._ble.write('\n')
            self._ble.write(self._accelerometer.toJson())
            self._ble.write('\n')
            self._ble.write(self._microphone.toJson())
            self._ble.write('\n')

    def onLogData(self, data, diskStatus):
        if diskStatus == 'Ok':
            self._leds.set(DSK_LED, (0,99,0))
        else:
            if diskStatus == 'Disk full':
                self._leds.set(DSK_LED, (99,40,0))
            else:
                self._leds.set(DSK_LED, (99,0,0))
        self._leds.clear(DSK_LED)


    def onBleConnectionStateChanged(self):
        if self._ble.connected:
            print("BLE Client connected")
            self._leds.set(CON_LED, (0,25,0))
        else:
            print("BLE Client disconnected")
            self._leds.clear(CON_LED)

    def onBleAdvertising(self):
        #self._leds.set(BLE_LED, (0, 255, 255))
        pass

    def AKeyPressed(self):
        #print("A")
        if self._gps.enabled:
            self._speaker.playFile("off.wav")
            self._gps.disable()            
            self._ble.stopAdvertising()
        else:
            self._speaker.playFile("on.wav")
            self._gps.enable()
            self._ble.startAdvertising()
            self._sensors.enable()
            self._accelerometer.enable()

    def BKeyPressed(self):        
        #print("B")
        self._ble.startAdvertising()

    def onWrite(self, data):        
        self._leds.set(BLE_LED, (0, 0, 155))
        self._leds.set(BLE_LED, (0, 0, 0))                

    def run(self):
        blinkTimer = Timer()
        gpsBootTimer = Timer()
        autoOffTimer = Timer()
        sendSensorDataTimer = Timer()
        sendAccelerometerDataTimer = Timer()
        #self._gps.enable()
        #self._ble.enable()        
        self._speaker.playFile("on.wav")

        didGPSBooted = False

        gpsBootTimer.stopWatch(3000)
        blinkTimer.stopWatch(30)
        sendSensorDataTimer.stopWatch(1000)
        sendAccelerometerDataTimer.stopWatch(1000)

        while True:
            self._buttons.read()

            if gpsBootTimer.enabled:
                if gpsBootTimer.stopped():
                    didGPSBooted =  True

            if didGPSBooted and self._gps.enabled:
                self._gps.read()

            """
            if not self._gps.enabled:
                if self._sensors.enabled:
                    if sendSensorDataTimer.stopped():
                        self.logSensorData()
                        sendSensorDataTimer.reset()

                if self._accelerometer.enabled:
                    if sendAccelerometerDataTimer.stopped():                    
                        self.logAccelerometerData()
                        sendAccelerometerDataTimer.reset()
            """
            if blinkTimer.enabled:
                if blinkTimer.stopped():
                    if self._ble.isAdvertising:
                        if not self._ble.connected:
                            self._leds.blink(CON_LED,(0,44,77))
                    blinkTimer.reset()

            self._ble.read()
            self._microphone.read()
