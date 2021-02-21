import analogio
import audiobusio
import storage
from adafruit_ble import BLERadio
import adafruit_thermistor
import math
import array
import adafruit_lis3dh
from adafruit_ble.services.nordic import UARTService
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
import time
import board
import neopixel
import struct
import busio
import digitalio
import binascii
import utils
import gc

FLAG_LOG_ON = True

GPS_LED = 0
SEN_LED = 1
ACC_LED = 2
MIC_LED = 3
DSK_LED = 4
BLE_LED = 5
CON_LED = 9

OFF_SCREEN = 0
GPS_SCREEN = 1
BLE_SCREEN = 2
MIC_SCREEN = 3
SENSOR_SCREEN = 4
ACC_SCREEN = 5
LOG_SCREEN = 6
LAST_SCREEN = 6

USEC_MULT = 1000
MSEC_MULT = 1000 * USEC_MULT
SECONDS_MULT = 1000 * MSEC_MULT
MIN_MULT = 60 * SECONDS_MULT
HOUR_MULT = 60 * MIN_MULT

RED = (255, 0, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
PINK = (255, 0, 100)
ORANGE = (230, 80, 0)
BLACK = (0, 0, 0)

TIMER_REPEAT = 0
TIMER_STOPWATCH = 1
TIMER_MARKER = 2


class UbxGps():

    def __init__(self, onDataHandler, bufferSize):
        self._onDataHandler = onDataHandler
        self._buffer = bytearray(bufferSize)
        self._ubx = b''
        self._uart = busio.UART(
            board.TX, board.RX, baudrate=9600, receiver_buffer_size=bufferSize)
        self._enablePin = digitalio.DigitalInOut(board.D9)
        self._enablePin.direction = digitalio.Direction.OUTPUT
        self.__ubxLeft = b""
        self.__ubxNext = b""
        self._satelliteCount = 0
        self.DATA_ERROR = {}
        self.VALID_MAG = False
        self.FULLY_RESOLVED = False
        self.VALID_TIME = False
        self.VALID_DATE = False
        self.GNSS_FIX_OK = False
        self.HEAD_VEH_VALID = False
        self.SAT_COUNT = False
        self._validationFlags = 0
        self._fixFlags = 0
        self.disable()

    def parseValidationFlags(self):
        validFlags = self._validationFlags
        fixFlags = self._fixFlags
        # print("{0:b}".format(flags))
        self.DATA_ERROR = {}
        if validFlags & 0b00001000:
            self.VALID_MAG = True
            self.DATA_ERROR['mag'] = True
        else:
            self.VALID_MAG = False
            self.DATA_ERROR['mag'] = False

        if validFlags & 0b00000100:
            self.FULLY_RESOLVED = True
            self.DATA_ERROR['resolved'] = True
        else:
            self.FULLY_RESOLVED = False
            self.DATA_ERROR['resolved'] = False

        if validFlags & 0b00000010:
            self.VALID_TIME = True
            self.DATA_ERROR['time'] = True
        else:
            self.VALID_TIME = False
            self.DATA_ERROR['time'] = False

        if validFlags & 0b00000001:
            self.VALID_DATE = True
            self.DATA_ERROR['date'] = True
        else:
            self.VALID_DATE = False
            self.DATA_ERROR['date'] = False

        if fixFlags & 0b10000000:
            self.DATA_ERROR['carsoln'] = True
        else:
            self.DATA_ERROR['carsoln'] = False

        if fixFlags & 0b00100000:
            self.HEAD_VEH_VALID = True
            self.DATA_ERROR['headvehvalid'] = True
        else:
            self.HEAD_VEH_VALID = False
            self.DATA_ERROR['headvehvalid'] = False

        if fixFlags & 0b00010000:
            self.DATA_ERROR['psmstate'] = True
        else:
            self.DATA_ERROR['psmstate'] = False

        if fixFlags & 0b00000010:
            self.DATA_ERROR['difsoln'] = True
        else:
            self.DATA_ERROR['difsoln'] = False

        if fixFlags & 0b00000001:
            self.GNSS_FIX_OK = True
            self.DATA_ERROR['gnssfixok'] = True
        else:
            self.GNSS_FIX_OK = False
            self.DATA_ERROR['gnssfixok'] = False

        if self._satelliteCount >= 3:
            self.SAT_COUNT = True
            self.DATA_ERROR['satcount'] = True
        else:
            self.SAT_COUNT = False
            self.DATA_ERROR['satcount'] = False

        # print(self.DATA_ERROR)

    def getSignalColor(self):
        maxLevel = 255

        mult = self._satelliteCount * 20

        sLevel = mult if mult < maxLevel else maxLevel

        if self.FULLY_RESOLVED:
            return (0, sLevel, 0)
        else:
            return (sLevel, 0, 0)

    def sendUbx(self, hexString):
        data = binascii.unhexlify(hexString.replace(' ', ''))
        for i in range(len(data)):
            print("{:02X} ".format(data[i]), end='')
        self._uart.write(data)

        drain = self._uart.read(100)

        time.sleep(1)

    def switchTo115200Bauds(self):
        # UBX Only
        #self.sendUbx("B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 03 00 03 00 00 00 00 00 BC 5E")
        self.sendUbx(
            "B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 07 00 03 00 00 00 00 00 C0 7E")

    def switchTo9600Bauds(self):
        self.sendUbx(
            "B5 62 06 00 14 00 01 00 00 00 C0 08 00 00 80 25 00 00 01 00 01 00 00 00 00 00 8A 79")

    def storeConfiguration(self):
        self.sendUbx(
            "B5 62 06 09 0D 00 00 00 00 00 02 00 00 00 00 00 00 00 03 21 CE")

    def getBasicGpsInfo(self):
        ubxMagic1 = self._ubx[0]
        ubxMagic2 = self._ubx[1]
        ubxClass = self._ubx[2]
        ubxId = self._ubx[3]
        ubxLen = self._ubx[4] + self._ubx[5] * 256
        packetLen = 6 + ubxLen + 2
        if len(self._ubx) != packetLen:
            self.DATA_ERROR = "xxxx"
        else:
            self._validationFlags = self._ubx[6 + 11]
            self._fixFlags = self._ubx[6 + 21]
            self.parseValidationFlags()
            self._satelliteCount = self._ubx[6 + 23]
            self.parseValidationFlags()

    def read(self):
        """Read Buffer from UART"""

        # if gps is disable do not read fro UART
        if not self._enablePin.value:
            return

        self._uart.readinto(self._buffer)

        magicIndex = self._buffer.find(b'\xb5\x62')
        if magicIndex >= 0:
            # move leftover into ubx and process it
            self._ubx += self._buffer[:magicIndex]

            # Get Basic Info
            self.getBasicGpsInfo()

            if self._onDataHandler:
                self._onDataHandler()

            # init ubx with the next bytes
            self._ubx = self._buffer[magicIndex:]
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
            print("Broken UBX: Expected {}  got {}".format(
                packetLen, len(self._ubx)))
            print(binascii.hexlify(self._ubx))
            json = {
                "type": "ubxNavPvt"
            }
            str = "{}".format(json)
            return str
        if ubxClass == 0x01 or ubxId == 0x07:
            x = struct.unpack_from(
                '<LHBBBBBBLLBBBBllllLLlllllLLHBBBBBBlhL', self._ubx, 6)
            json = {
                "type": "ubxNavPvt",
                "valid": self.DATA_ERROR,
                "numsv": x[13],
                "lon": x[14], "lat": x[15], "alt": x[16], "hmsl": x[17], "hacc": x[18], "vacc": x[19],
                "itow": x[0],
                "year": x[1], "month": x[2], "day": x[3], "hour": x[4], "min": x[5], "sec": x[6],
                "valid": x[7], "tacc": x[8], "nano": x[9], "fixtype": x[10], "flags": x[11], "flags2": x[12],
                "veln": x[20], "vele": x[21], "veld": x[22], "speed": x[23],
                "headmot": x[24], "sacc": x[25], "headacc": x[26], "pdop": x[27], "flags3": x[28], "rsvd1": x[29], "headveh": x[30],
                "magdec": x[31], "magacc": x[22]
            }
            str = "{}".format(json)
        return str

    def enable(self):
        self._enablePin.value = True
        print("GPS ON" if self._enablePin.value else "GPS OFF")

    def disable(self):
        self._enablePin.value = False

    def toggle(self):
        if self._enablePin.value:
            self.disable()
        else:
            self.enable()

    @property
    def enabled(self):
        return self._enablePin.value


class Leds():
    def __init__(self, pixelCount, brightness):
        self._pixels = neopixel.NeoPixel(
            board.NEOPIXEL, pixelCount, brightness=brightness, auto_write=False)
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
        self._pixels[id] = (0, 0, 0)
        self._pixels.show()

    def togle(self, id, color):
        toggles[id] = not toggles[id]
        if toggles[id]:
            self._pixels[id] = color
        else:
            self._pixels[id] = (0, 0, 0)
        self._pixels.show()


class Ble():
    def __init__(self, onConnectionStateChanged, onAdvertising=None, onWrite=None):
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
        self._enabled = False

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
        # if self._ble.connected:
        print("Stopped Advertising")
        self._ble.stop_advertising()
        self._isAdvertising = False

    def read(self):

        if self._ble.connected != self.__oldConnectionState:
            self._onConnectionStateChanged()

        self.__oldConnectionState = self._ble.connected

    def enable(self):
        self.startAdvertising()
        self._enabled = True

    def disable(self):
        self.stopAdvertising()
        self._enabled = False

    def toggle(self):
        if self._enabled:
            self.disable()
        else:
            self.enable()

    @property
    def isAdvertising(self):
        return self._isAdvertising

    @property
    def connected(self):
        return self._ble.connected

    @property
    def enabled(self):
        return self._enabled


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
        self._buttonAState = False
        self._buttonBState = False
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


class SimpleLogger:
    """Basic logging on the local storage"""

    def __init__(self, onLogData, header, logs, logSize):
        self._onLogData = onLogData
        self._logFile = "0.pubx"
        self._logs = logs
        self._crtLog = 0
        self._logSize = logSize
        self._writable = True
        self._diskFull = False
        self._loggedBytes = 0
        self._header = header
        self._diskState = "Ok"
        self._debug = True
        self._enabled = False
        self.testDiskAccess()

    def testDiskAccess(self):
        """Appends data to a file"""
        try:
            with open("test.txt", "w") as fp:
                fp.write("ok")
                fp.flush()
        except OSError as e:
            # cant write -> not writable
            if e.args[0] == 28:  # If the file system is full...
                self._diskState = "Disk full"
                self._diskFull = True
            else:
                self._diskState = "Disk protected"
            self._writable = False

    def checkNextFile(self):
        # check if log file size exceeds the allowed max size
        if self._loggedBytes > self._logSize:

            # increment log number
            self._crtLog = self._crtLog + 1

            # use circular logging
            if self._crtLog >= self._logs:
                self._crtLog = 0

            # build the filename
            self._logFile = "{}.pubx".format(self._crtLog)

            if self._writable:
                # initialize the new file and write the header befor logging
                self.writeData(self._header, 'w')

            # init the logged bytes to 0
            self._loggedBytes = 0

    def writeData(self, data, flag):
        # write data
        if self._writable:
            with open(self._logFile, flag) as fp:
                fp.write(data)
                fp.flush()
                self._loggedBytes = self._loggedBytes + len(data)
        else:
            if self._debug:
                print("{} {} [{}/{}]".format(self._diskState,
                                             self._logFile, len(data), self._loggedBytes))
                # print(data)
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
                self._diskFull = True
            else:
                self._diskState = "Disk protected"
            self._writable = False

    def appendLine(self, data):
        """Appends a line of data to a file"""
        self.append(data+"\n")

    def enable(self):
        print("Logging On")
        self._enabled = True

    def disable(self):
        print("Logging Off")
        self._enabled = False

    def toggle(self):
        if self._enabled:
            self.disable()
        else:
            self.enable()

    @property
    def enabled(self):
        return self._enabled

    @property
    def canWrite(self):
        return self._writable


class TimerPool():
    def __init__(self):
        self._name = []
        self._startTime = []
        self._delta = []
        self._time = []
        self._onTime = []
        self._enabled = []
        self._type = []

    def addTimer(self, name, type, setTime, onTime, enabled):
        crtTime = time.monotonic_ns()
        id = len(self._name)

        self._name.append(name)
        self._type.append(type)
        self._startTime.append(crtTime)
        self._delta.append(0)
        self._time.append(setTime)
        self._onTime.append(onTime)
        self._enabled.append(enabled)

        return id

    def getTimerId(self, name):
        for id in range(len(self._name)):
            if self._name[id] == name:
                return id
        else:
            return -1

    def restart(self, id):
        self._startTime[id] = time.monotonic_ns()
        self._delta[id] = 0
        self._enabled[id] = True

    def stop(self, id):
        self._startTime[id] = time.monotonic_ns()
        self._delta[id] = 0
        self._enabled[id] = False

    def resume(self, id):
        self._enabled[id] = True

    def restartByName(self, name):
        self.restart(self.getTimerId(name))

    def changeHandler(self, id, newHandler):
        self.stop(id)
        self._onTime[id] = newHandler
        self.resume(id)

    def changeTime(self, id, newTime):
        self.stop(id)
        self._time[id] = newTime
        self.restart(id)

    def tick(self):
        crtTime = time.monotonic_ns()

        for id in range(len(self._name)):

            self._delta[id] = crtTime - self._startTime[id]

            if self._delta[id] >= self._time[id]:

                if self._enabled[id]:
                    self._onTime[id]()

                # Repeat Timer
                if self._type[id] == TIMER_REPEAT:
                    self._startTime[id] = time.monotonic_ns()
                    self._delta[id] = 0

                # StopWatch Timer
                if self._type[id] == TIMER_STOPWATCH:
                    # Disable
                    self._delta[id] = 0
                    self._enabled[id] = False


class Accelerometer():
    """"""

    def __init__(self):
        if hasattr(board, "ACCELEROMETER_SCL"):
            self._acc_i2c = busio.I2C(
                board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
            self._lis3dh = adafruit_lis3dh.LIS3DH_I2C(
                self._acc_i2c, address=0x19)
        else:
            self._acc_i2c = busio.I2C(board.SCL, board.SDA)
            self._lis3dh = adafruit_lis3dh.LIS3DH_I2C(self._acc_i2c)
        # Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
        self._lis3dh.range = adafruit_lis3dh.RANGE_8_G
        self._lis3dh.set_tap(2, 60)
        self._enabled = False
        self._data = [0, 0, 0]

    def read(self):

        if not self._enabled:
            return

        #self._data = [ value / adafruit_lis3dh.STANDARD_GRAVITY for value in self._lis3dh.acceleration]
        # for i in range(3):
        #    self._data[i] = self._lis3dh.acceleration[i]
        self._data = self._lis3dh.acceleration

    def toUbx(self):
        return struct.pack("<BBBBHfff", 0xb5, 0x62, ord('S'), ord('A'), 4 * 3, self._data[0], self._data[1], self._data[2])

    def toJson(self):
        json = {
            'accx': self._data[0],
            'accy': self._data[1],
            'accz': self._data[2]
        }
        return "{}".format(json)

    def enable(self):
        print("Accelerometer On")
        self._enabled = True

    def disable(self):
        print("Accelerometer Off")
        self._enabled = False

    def toggle(self):
        if self._enabled:
            self.disable()
        else:
            self.enable()

    @property
    def enabled(self):
        return self._enabled


class Microphone():
    """ Samples data from the microphone and returns the noise
        magnitude and level"""

    def __init__(self, numberOfSamples=64):
        self._NUM_SAMPLES = numberOfSamples
        self._SCALE_EXPONENT = math.pow(10, 2 * -0.1)  # curve is 2
        self._mic = audiobusio.PDMIn(
            board.MICROPHONE_CLOCK, board.MICROPHONE_DATA, sample_rate=16000, bit_depth=16)
        # Record an initial sample to calibrate. Assume it's quiet when we start.
        self._samples = array.array('H', [0] * self._NUM_SAMPLES)
        self._mic.record(self._samples, len(self._samples))
        # Set lowest level to expect, plus a little.
        self._input_floor = self.normalized_rms(self._samples) + 10
        self._input_ceiling = self._input_floor + 500
        self._level = 0
        self._magnitude = 0
        self._enabled = False

    def constrain(self, value, floor, ceiling):
        """Restricts value to be between floor and ceiling."""
        return max(floor, min(value, ceiling))

    def log_scale(self, input_value, input_min, input_max, output_min, output_max):
        """Scale input_value between output_min and output_max, exponentially."""
        normalized_input_value = (
            input_value - input_min) / (input_max - input_min)
        return output_min + math.pow(normalized_input_value, self._SCALE_EXPONENT) * (output_max - output_min)

    def normalized_rms(self, values):
        """Remove DC bias before computing RMS."""
        minbuf = int(self.mean(values))
        samples_sum = sum(float(sample - minbuf) * (sample - minbuf)
                          for sample in values)
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
        self._level = self.log_scale(self.constrain(
            self._magnitude, self._input_floor, self._input_ceiling), self._input_floor, self._input_ceiling, 0, 255)

    def toJson(self):
        json = {
            "magnitude": self._magnitude, "level": self._level
            # , "samples" : self._samples
        }
        return "{}".format(json)

    def toUbx(self):
        return struct.pack("<BBBBHff", 0xb5, 0x62, ord('S'), ord('M'), 4 * 2, self._magnitude/100, self._level/100)

    def enable(self):
        print("Microphone On")
        self._enabled = True

    def disable(self):
        print("Microphone Off")
        self._enabled = False

    def toggle(self):
        if self._enabled:
            self.disable()
        else:
            self.enable()

    @property
    def enabled(self):
        return self._enabled


class Sensors():
    """Sensors Class"""

    def __init__(self):
        # Configure the Thermistor for reading the temperature
        self._thermistor = adafruit_thermistor.Thermistor(
            board.TEMPERATURE, 10000, 10000, 25, 3950)
        self._light = analogio.AnalogIn(board.LIGHT)
        self._enabled = False
        self._data = [0, 0, 0]

    def scale(self, value):
        """Scale the light sensor values from 0-65535 (AnalogIn range)
        to 0-50 (arbitrarily chosen to plot well with temperature)"""
        return (value / 65535 * 50) * 10

    def read(self):

        if not self._enabled:
            return

        self._data[0] = self._thermistor.temperature
        self._data[1] = self.scale(self._light.value)

    def toUbx(self):
        return struct.pack("<BBBBHff", 0xb5, 0x62, ord('S'), ord('S'), 4 * 2, self._data[0], self._data[1])

    def toJson(self):
        json = {
            'temp': self._data[0],
            'light': self._data[1]
        }
        return "{}".format(json)

    def enable(self):
        print("Sensors On")
        self._enabled = True

    def disable(self):
        print("Sensors Off")
        self._enabled = False

    def toggle(self):
        if self._enabled:
            self.disable()
        else:
            self.enable()

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
        self._ble = Ble(self.onBleConnectionStateChanged,
                        self.onBleAdvertising, self.onWrite)
        self._speaker = Speaker()
        self._buttons = Buttons(self.AKeyPressed, self.BKeyPressed)
        self._sensors = Sensors()
        self._accelerometer = Accelerometer()
        self._microphone = Microphone(32)
        self._logger = SimpleLogger(self.onLogData, b'UBX', 10, 128 * 1024)
        self._screenId = 0
        self._timers = TimerPool()

    def handleScreens(self):
        self.handleAllScreens()
        if self._screenId == OFF_SCREEN:
            self.handleOffScreen()
        if self._screenId == GPS_SCREEN:
            self.handleGpsScreen()
        if self._screenId == BLE_SCREEN:
            self.handleBleScreen()
        if self._screenId == SENSOR_SCREEN:
            self.handleSensorScreen()
        if self._screenId == ACC_SCREEN:
            self.handleAccScreen()
        if self._screenId == MIC_SCREEN:
            self.handleMicScreen()
        if self._screenId == LOG_SCREEN:
            self.handleLogScreen()
        self._leds.allOff()  # saves energy

    def currenScreenIs(self, screeId):
        return self._screenId == screeId

    def nextScreen(self):
        self._screenId += 1
        if self._screenId > LAST_SCREEN:
            self._screenId = 0
        return self._screenId

    def setScreen(self, screenId):
        self._screenId = screenId
        self._speaker.playFile("on.wav")

        self.handleAllScreens()

        if self._screenId == OFF_SCREEN:
            print("All Feature")
        if self._screenId == GPS_SCREEN:
            print("GPS")
        if self._screenId == BLE_SCREEN:
            print("BLE")
        if self._screenId == SENSOR_SCREEN:
            print("Sensors")
        if self._screenId == ACC_SCREEN:
            print("Accelerometer")
        if self._screenId == MIC_SCREEN:
            print("Microphone")
        if self._screenId == LOG_SCREEN:
            print("Logger")

    def handleAllScreens(self):
        if not self._logger._writable:
            self._leds.set(0, RED)
        if self._logger._diskFull:
            self._leds.set(0, ORANGE)

    def handleOffScreen(self):
        pass

    def handleGpsScreen(self):
        if not self._gps.enabled:
            return

        self._leds.set(2, GREEN if self._gps.FULLY_RESOLVED else RED)
        self._leds.set(3, GREEN if self._gps.SAT_COUNT else RED)
        self._leds.set(4, GREEN if self._gps.HEAD_VEH_VALID else RED)
        self._leds.set(5, GREEN if self._gps.GNSS_FIX_OK else RED)
        self._leds.set(6, GREEN if self._gps.VALID_TIME else RED)
        self._leds.set(7, GREEN if self._gps.VALID_DATE else RED)
        self._leds.set(9, self._gps.getSignalColor())
        # self._leds.allOff()
        #self._leds.set(3, self._gps.getSignalColor())
        # self._gps.switchTo115200Bauds()
        # self._gps.storeConfiguration()

    def handleBleScreen(self):
        if not self._ble.enabled:
            return

        self._leds.set(3, BLUE if self._ble._isAdvertising else RED)
        self._leds.set(4, BLUE if self._ble.connected else RED)

    def handleSensorScreen(self):
        if not self._sensors.enabled:
            return

    def handleAccScreen(self):
        if not self._accelerometer.enabled:
            return

    def handleMicScreen(self):
        if not self._microphone.enabled:
            return

    def handleLogScreen(self):
        if not self._logger.enabled:
            return

    def sendAllData(self):
        """Send All Data"""

        if self._logger.enabled:
            self._logger.append(self._gps.toUbx())

        if self._sensors.enabled:
            self._sensors.read()

        if self._logger.enabled:
            self._logger.append(self._sensors.toUbx())

        self._buttons.read()

        if self._accelerometer.enabled:
            self._accelerometer.read()

        if self._logger.enabled:
            self._logger.append(self._accelerometer.toUbx())

        if self._microphone.enabled:
            self._microphone.read()

        if self._logger.enabled:
            self._logger.append(self._microphone.toUbx())

        self._buttons.read()

        if self._ble.enabled:
            if self._ble.connected:
                if self._gps.enabled:
                    self._ble.write(self._gps.toJson())
                    self._ble.write('\n')
                if self._sensors.enabled:
                    self._ble.write(self._sensors.toJson())
                    self._ble.write('\n')
                if self._accelerometer.enabled:
                    self._ble.write(self._accelerometer.toJson())
                    self._ble.write('\n')
                if self._microphone.enabled:
                    self._ble.write(self._microphone.toJson())
                    self._ble.write('\n')

    def onLogData(self, data, diskStatus):
        pass

    def onBleConnectionStateChanged(self):
        if self._ble.connected:
            print("BLE Client connected")
        else:
            print("BLE Client disconnected")

    def onBleAdvertising(self):
        pass

    def AKeyPressed(self):
        # print("A")
        if self.currenScreenIs(OFF_SCREEN):
            if self._gps.enabled:
                self._speaker.playFile("off.wav")
                self._ble.stopAdvertising()
                self._gps.disable()
                self._ble.disable()
                self._sensors.disable()
                self._accelerometer.disable()
                self._microphone.disable()
                self._logger.disable()
            else:
                self._speaker.playFile("on.wav")
                self._ble.startAdvertising()
                self._gps.enable()
                self._ble.enable()
                self._sensors.enable()
                self._accelerometer.enable()
                self._microphone.enable()
                self._logger.enable()

        if self.currenScreenIs(GPS_SCREEN):
            if self._gps.enabled:
                self._speaker.playFile("off.wav")
                self._gps.disable()
            else:
                self._speaker.playFile("on.wav")
                self._gps.enable()

        if self.currenScreenIs(BLE_SCREEN):
            if self._ble.enabled:
                self._speaker.playFile("off.wav")
                self._ble.disable()
            else:
                self._speaker.playFile("on.wav")
                self._ble.enable()

        if self.currenScreenIs(SENSOR_SCREEN):
            if self._sensors.enabled:
                self._speaker.playFile("off.wav")
                self._sensors.disable()
            else:
                self._speaker.playFile("on.wav")
                self._sensors.enable()

        if self.currenScreenIs(ACC_SCREEN):
            if self._accelerometer.enabled:
                self._speaker.playFile("off.wav")
                self._accelerometer.disable()
            else:
                self._speaker.playFile("on.wav")
                self._accelerometer.enable()

        if self.currenScreenIs(MIC_SCREEN):
            print("MICROPHONE")
            if self._microphone.enabled:
                self._speaker.playFile("off.wav")
                self._microphone.disable()
            else:
                self._speaker.playFile("on.wav")
                self._microphone.enable()

        if self.currenScreenIs(LOG_SCREEN):
            print("LOGGER")
            if self._logger.enabled:
                self._speaker.playFile("off.wav")
                self._logger.disable()
            else:
                self._speaker.playFile("on.wav")
                self._logger.enable()

    def BKeyPressed(self):
        # print("B")
        self.setScreen(self.nextScreen())
        self._timers.restartByName("Standby")

    def onWrite(self, data):
        pass

    def onTickSecond(self):
        self.handleScreens()

    def onTickMinute(self):
        if self.checkFreeMem() < 20 * 1024:
            gc.collect()
            print("{:.2f} kb free after gc.collect()".format(
                self.checkFreeMem()/1024))

    def onTickHour(self):
        pass

    def onStandby(self):
        if self._screenId != OFF_SCREEN:
            self.setScreen(OFF_SCREEN)

    def onTick50Ms(self):
        if self._screenId != OFF_SCREEN:
            self._leds.blink(self._screenId, BLUE)

    def checkFreeMem(self):
        freeMem = gc.mem_free()
        print("{:.2f} kb free".format(freeMem/1024))
        return freeMem

    def registerTimers(self):
        self._timers.addTimer("Second", TIMER_REPEAT, 1 *
                              SECONDS_MULT, self.onTickSecond, True)
        self._timers.addTimer("Standby", TIMER_STOPWATCH,
                              30000*SECONDS_MULT, self.onStandby, True)
        self._timers.addTimer("Minute", TIMER_REPEAT, 1 *
                              MIN_MULT, self.onTickMinute, True)
        self._timers.addTimer("Hour", TIMER_REPEAT, 1 *
                              HOUR_MULT, self.onTickHour, True)
        self._timers.addTimer("50msec", TIMER_REPEAT,
                              100*MSEC_MULT, self.onTick50Ms, True)

    def run(self):
        self._speaker.playFile("on.wav")
        self.registerTimers()
        self.checkFreeMem()
        while True:
            self._buttons.read()
            self._gps.read()
            self._buttons.read()
            self._ble.read()
            self._buttons.read()
            self._microphone.read()
            self._buttons.read()
            self._buttons.read()
            self._timers.tick()
