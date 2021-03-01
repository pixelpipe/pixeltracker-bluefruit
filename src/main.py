""" Pixeltracker Library """
import time
import supervisor
import board
import busio
import analogio
import audiobusio
import digitalio
import binascii
import struct
import storage
import neopixel
import math
import array
import gc
import os
import adafruit_thermistor
import adafruit_lis3dh
from adafruit_ble import BLERadio
from adafruit_ble.services.nordic import UARTService
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement

DEBUG = False
JSON_TO_BLE = True
SIM_VALID_GPS_DATA = False
ON_BAT = False

RED = (255, 0, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
PINK = (255, 0, 100)
ORANGE = (230, 80, 0)
BLACK = (0, 0, 0)


def display(text):
    if supervisor.runtime.serial_connected:
        print(text)


class UbxGps():
    """ GPS UBX Class """

    def __init__(self, onDataHandler, bufferSize):
        self._onDataHandler = onDataHandler
        self._ubx = b''
        self._oldByte = ''
        self._newByte = ''
        self._ubxClass = 0xff
        self._ubxId = 0xff
        self._validClassId = False
        self._validPayload = False
        self._uart = busio.UART(
            board.TX, board.RX, baudrate=9600, receiver_buffer_size=bufferSize)
        self._enablePin = digitalio.DigitalInOut(board.D9)
        self._enablePin.direction = digitalio.Direction.OUTPUT
        self._changeFileName = False
        self._signalOk = False
        self.disable()

    def read(self):
        """Read GPS message char by char"""
        # if gps is disabled do not read fro UART
        if not self._enablePin.value:
            return
        buffer = self._uart.read(1)
        # No data no processing
        if buffer == None:
            return
        # Empty buffer, no read
        if len(buffer) == 0:
            return
        # Got new byte
        self._newByte = buffer[0]
        # Check ubx magic
        if self._oldByte == 0xb5 and self._newByte == 0x62:
            # put the magic back in front
            self._ubx = b'\xb5' + self._ubx
            self._ubx = self._ubx[0:-1]
            if SIM_VALID_GPS_DATA:
                self._ubx = self.generateSimulatedGpsUbx()
            # display(f"{binascii.hexlify(self._ubx)}")
            # Validate the Ubx packet
            self.parseUbx()
            # Call the data handler if the ubx packet is valid
            if self._validClassId and self._validPayload:
                self.parseValidationInfo()
                if self._onDataHandler:
                    self._onDataHandler()
            # Clean the ubx buffer
            self._ubx = bytearray()
        # Append buffer to the ubx buffer
        self._ubx += buffer
        # make the new byte old
        self._oldByte = self._newByte

    def getTimeStamp(self):
        json = {}
        self.toJson(json)
        if DEBUG:
            display(json)
        return json["timestamp"]

    def parseUbx(self):
        """Parse the ubx header by checking the class, id and the payload len"""
        ubxMagic1 = self._ubx[0]
        ubxMagic2 = self._ubx[1]
        self._ubxClass = self._ubx[2]
        self._ubxId = self._ubx[3]
        payloadLen = self._ubx[4] + self._ubx[5] * 256
        # Header[6] + PayloadLen + Checksum[2]
        calculatedUbxLength = 6 + payloadLen + 2

        if SIM_VALID_GPS_DATA:
            calculatedUbxLength = calculatedUbxLength + 32  # add sensor data

        if self._ubxClass == 0x01 and self._ubxId == 0x07:
            self._validClassId = True
        # add more elif here
        else:
            self._validClassId = False
            if DEBUG:
                display("Unknown ClassId")

        self._validPayload = len(self._ubx) == calculatedUbxLength
        if DEBUG:
            if not self._validPayload:
                display("Invalid Payload. Expected {}. Calculated {}".format(
                    len(self._ubx), calculatedUbxLength))

    def parseValidationInfo(self):
        vflags = self._ubx[6 + 11]
        fflags = self._ubx[6 + 21]
        self._validationResults = {
            "satcount": self._ubx[6 + 23],
            "pdop": (self._ubx[6 + 76] + self._ubx[6 + 77] * 256) / 100,
            "validllh": (self._ubx[6 + 78] & 0b00000001) == 0,
            "validmag": vflags & 0b00001000 != 0,
            "fullyresolved": vflags & 0b00000100 != 0,
            "validtime": vflags & 0b00000010 != 0,
            "validdate": vflags & 0b00000001 != 0,
            "carsoln": fflags & 0b10000000 != 0,
            "hedvehvalid": vflags & 0b00100000 != 0,
            "psmstate": vflags & 0b00010000 != 0,
            "difsoln": vflags & 0b00000010 != 0,
            "gnssfixok": vflags & 0b00000001 != 0
        }

        accurate = self._validationResults["pdop"] < 4
        # and self._validationResults["validtime"] and self._validationResults["fullyresolved"]
        timeok = self._validationResults["validdate"]
        if DEBUG:
            display("Valid Date, Time and FullyResolved")
            display(self._validationResults["validdate"])
            display(self._validationResults["validtime"])
            display(self._validationResults["fullyresolved"])

        if timeok and self._justStarted:
            self._changeFileName = True

        validcoordinates = self._validationResults["validllh"]

        if DEBUG:
            display(
                "Enabled, ValidClassId, ValidPayload, Accurate, TimeOK and ValidCoordinates")
            display(self.enabled)
            display(self._validClassId)
            display(self._validPayload)
            display(accurate)
            display(timeok)
            display(validcoordinates)

        self._signalOk = self.enabled and self._validClassId and self._validPayload and accurate and timeok and validcoordinates

    def toHex(self):
        """Return ubx as hex string data"""
        return binascii.hexlify(self._ubx)

    def toUbx(self):
        """Return the current ubx as binary data"""
        return self._ubx

    def toJson(self, json):
        if self._ubxClass == 0x01 or self._ubxId == 0x07:
            X = struct.unpack_from(
                '<LHBBBBBBLLBBBBllllLLlllllLLHBBBBBBlhHH', self._ubx, 6)
            date_time_str = f'{X[3]:02d}/{X[2]:02d}/{X[1]} {X[4]:02d}:{X[5]:02d}:{X[6]:02d}'
            timestamp = f'{X[1]}-{X[2]:02d}-{X[3]:02d}T{X[4]:02d};{X[5]:02d};{X[6]:02d}'
            json["datetime"] = date_time_str
            json["timestamp"] = timestamp
            #json["satcount"] = X[13]
            json["longitude"] = X[14] / 10000000
            json["latitude"] = X[15] / 10000000
            json["height"] = X[16] / 1000
            #json["hmsl"] = X[17]
            #json["horacc"] = X[18] / 1000
            #json["veracc"] = X[19] / 1000
            #json["itow"] = X[0]
            #json["year"] = X[1]
            #json["month"] = X[2]
            #json["day"] = X[3]
            #json["hour"] = X[4]
            #json["min"] = X[5]
            #json["sec"] = X[6]
            #json["valid"] = X[7]
            #json["timeacc"] = X[8]
            json["ns"] = X[9]
            #json["fixtype"] = X[10]
            #json["fixstatustype"] = X[11]
            #json["AdditionalFlags2"] = X[12]
            #json["NEDVelocityNorth"] = X[20] * 3.6 / 1000
            #json["NEDVelocityEast"] = X[21] * 3.6 / 1000
            #json["NEDVelocityDown"] = X[22] * 3.6 / 1000
            json["speed"] = X[23] * 3.6 / 1000,
            json["heading"] = X[24]
            #json["SpeedAccuracy"] = X[25] * 3.6 / 1000
            #json["HeadAccuracy"] = X[26]
            json["pdop"] = X[27] / 100
            #json["AdditionalFlags3"] = X[28]
            #json["Reserved1"] = X[29]
            json["headingveh"] = X[30]
            #json["MagneticDeclination"] = X[31]
            #json["MagneticDeclinationAccuracy"] = X[22]
            #json["Checksum"] = X[32]

    def generateSimulatedGpsUbx(self):
        return b"\xB5\x62\x01\x07\x5C\x00\x98\xEE\x6A\x13\xE5\x07\x02\x18\x12\x1D\x11\xF7\xF8\x03\x00\x00\x8D\xF2\xFD\xFF\x03\x01\x0A\x0C\x08\xF1\x04\x05\x26\x64\x38\x1D\x84\x59\x02\x00\x25\xA0\x01\x00\x8F\x15\x00\x00\x22\x1C\x00\x00\xE3\x2A\x00\x00\x02\x33\x00\x00\x2D\x01\x00\x00\xA4\x42\x00\x00\x2A\x35\x4C\x00\xA9\x02\x00\x00\xE7\xFD\x02\x00\x8C\x00\x00\x00\xE0\x4A\x23\x00\x00\x00\x00\x00\x00\x00\x00\x00\x07\xD3\x45\x31\x1C\x00\x60\xE5\x9C\x3D\xBC\xF4\xFE\xBF\x8C\x06\x10\x41\xCC\x8D\x20\x3F\x00\x00\x00\x00\x00\x6C\xAB\x41\x00\x00\x00\x00"
        """
        return struct.pack(
            "<BBBBH"+"LHBBBBB"+"B"+"LL"+"BBBB"+"lll"+"lLLlllllLLHBBBBBBlhHH",
            0xb5,0x62,0x01,0x07,0x5c,
            0x12345678,0x2021,0x02,0x03,0x04,0x45,0x32, #Time
            0b00001111, # All valid
            1.3276, 0x12345678,
            2, 0b10110011, 0b11100000, 12, # fix flags and SatCount
            49.12345 * 10000000, 8.12345 * 10000000, 120 * 1000,

            )
        """

    def convertHexDumpToBytearray(self, hexString):
        data = binascii.unhexlify(hexString.replace(' ', ''))
        display(data)
        """
        for i in range(len(data)):
            display("{:02X} ".format(data[i]), end='')
        """
        return data

    def sendUbx(self, hexString):
        data = self.convertHexDumpToBytearray(hexString)
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

    def enable(self):
        self._enablePin.value = True
        self._justStarted = True
        display("GPS ON" if self._enablePin.value else "GPS OFF")

    def disable(self):
        self._enablePin.value = False
        self._justStarted = False

    def toggle(self):
        if self._enablePin.value:
            self.disable()
        else:
            self.enable()

    @property
    def enabled(self):
        return self._enablePin.value

    @property
    def signalOk(self):
        return self._signalOk


class Leds():
    def __init__(self, pixelCount=10, brightness=0.01):
        self._pixels = neopixel.NeoPixel(
            board.NEOPIXEL, pixelCount, brightness=brightness, auto_write=False)
        self.toggles = [False] * pixelCount

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
        self.toggles[id] = not self.toggles[id]
        if self.toggles[id]:
            self._pixels[id] = color
        else:
            self._pixels[id] = (0, 0, 0)
        self._pixels.show()

    def showPeak(self, led, level, mid, max):
        if level <= mid:
            color = (0, level, 0)
        elif level > mid and level <= max:
            color = (level, level, 0)
        elif level > max:
            color = (level, 0, 0)
        self.set(led, color)


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
        display("Start Phone App and connect")
        display("Started Advertising to BLE devices")
        if self._onAdvertising:
            self._onAdvertising()
        self._isAdvertising = True

    def stopAdvertising(self):
        # if self._ble.connected:
        display("Stopped Advertising")
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
        self._logFile = "default.pubx"
        self._logs = logs
        self._crtLog = 0
        self._logSize = logSize
        self._writable = False
        self._diskFull = False
        self._loggedBytes = 0
        self._header = header
        self._diskState = "Ok"
        self._enabled = False
        self._debug = DEBUG

    def testDiskAccess(self):
        """Appends data to a file"""
        try:
            with open("test.txt", "w") as fp:
                fp.write("ok")
                fp.flush()
                display("Disk OK")
                self._diskState = "Disk OK"
                if self._onLogData:
                    self._onLogData("ok", self._diskState)
                self._writable = True
        except OSError as e:
            # cant write -> not writable
            if e.args[0] == 28:  # If the file system is full...
                self._diskState = "Disk full"
                self._diskFull = True
            else:
                self._diskState = "Disk protected"
            display(self._diskState)
            if self._onLogData:
                self._onLogData("ok", self._diskState)
            self._writable = False

    def deleteLogs(self):
        if not self._writable:
            if DEBUG:
                print("Disk is protected")
            return

        if DEBUG:
            print("Removing logs...")

        files = os.listdir()
        for file in files:
            if file.endswith(".pubx"):
                if DEBUG:
                    print("Removing {}".format(file))
                os.remove(file)

    def changeLogFileName(self, timestamp):
        if self._debug:
            display("Changing log filename to {}".format(timestamp))
        self._logFile = "{}.pubx".format(timestamp)

        if self._writable:
            # initialize the new file and write the header befor logging
            self.writeData(self._header, 'w')
            self._diskState = "New File : "+self._logFile
            if self._onLogData:
                self._onLogData(self._header, self._diskState)

        self._loggedBytes = 0

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
                self._diskState = "New File : "+self._logFile
                if self._onLogData:
                    self._onLogData(self._header, self._diskState)

            # init the logged bytes to 0
            self._loggedBytes = 0

    def writeData(self, data, flag):
        # write data
        if self._writable:
            with open(self._logFile, flag) as fp:
                if self._debug:
                    display(f"Writing into {self._logFile} using {flag} mode")
                fp.write(data)
                fp.flush()
                self._diskState = "Disk Write"
                if self._onLogData:
                    self._onLogData(data, self._diskState)
                self._loggedBytes = self._loggedBytes + len(data)
        else:
            if self._debug:
                display("{} {} [{}/{}]".format(self._diskState,
                                               self._logFile, len(data), self._loggedBytes))
                # display(data)
            self._loggedBytes = self._loggedBytes + len(data)

    def append(self, data):
        """Appends data to a file"""
        try:
            # logs only if writable, otherwise sends to stdio
            # self.checkNextFile()
            self.writeData(data, 'a')
        except OSError as e:
            # cant write -> not writable
            if e.args[0] == 28:  # If the file system is full...
                self._diskState = "Disk full"
                self._diskFull = True
            else:
                self._diskState = "Disk protected"
            self._writable = False
            if self._onLogData:
                self._onLogData(data, self._diskState)

    def appendLine(self, data):
        """Appends a line of data to a file"""
        self.append(data+"\n")

    def enable(self):
        display("Logging On")
        self._enabled = True

    def disable(self):
        display("Logging Off")
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
        return struct.pack("<fff", self._data[0], self._data[1], self._data[2])

    def toJson(self, json):
        json['gravityx'] = self._data[0]
        json['gravityy'] = self._data[1]
        json['gravityz'] = self._data[2]

    def enable(self):
        display("Accelerometer On")
        self._enabled = True

    def disable(self):
        display("Accelerometer Off")
        self._enabled = False

    def toggle(self):
        if self._enabled:
            self.disable()
        else:
            self.enable()

    @property
    def enabled(self):
        return self._enabled


# Imports for speaker
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

    def toJson(self, json):
        json["micnoise"] = self._magnitude
        json["miclevel"] = self._level

    def toUbx(self):
        return struct.pack("<ff", self._magnitude/100, self._level/100)

    def enable(self):
        display("Microphone On")
        self._enabled = True

    def disable(self):
        display("Microphone Off")
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
        return struct.pack("<ff", self._data[0], self._data[1])

    def toJson(self, json):
        json['temperature'] = self._data[0]
        json['light'] = self._data[1]

    def enable(self):
        display("Sensors On")
        self._enabled = True

    def disable(self):
        display("Sensors Off")
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


class Pixeltracker():
    """App Class"""

    def __init__(self):
        self._gps = UbxGps(self.sendAllData, 120)
        self._leds = Leds(10, 0.05)
        self._ble = Ble(self.onBleConnectionStateChanged,
                        self.onBleAdvertising, self.onWrite)
        self._speaker = Speaker()
        self._buttons = Buttons(self.AKeyPressed, self.BKeyPressed)
        self._sensors = Sensors()
        self._accelerometer = Accelerometer()
        self._microphone = Microphone(32)
        self._logger = SimpleLogger(self.onLogData, b'UBX', 10, 128 * 1024)
        self._speaker.playFile("on.wav")
        self.checkFreeMem()
        self._logger.testDiskAccess()
        self._LOGGER_LED = 0
        self._BLE_LED = 1
        self._MIC_LED = 2
        self._GPS_DATA_COUNTER_IN_SECONDS = 0
        self._LOG_INTERVAL_IN_SECONDS = 1
        self._SCREEN = 10
        self._GPS_SCREEN = 0
        self._BLE_SCREEN = 1
        self._MIC_SCREEN = 2
        self._DISK_SCREEN = 9
        self._OFFSCREEN = 10

    def extendUbx(self, count):
        return struct.pack("<BBH", ord('E'), ord('1'), count)

    def buildLogData(self):
        logData = self._gps.toUbx()
        logData += self.extendUbx(12+8+8)
        logData += self._accelerometer.toUbx()
        logData += self._microphone.toUbx()
        logData += self._sensors.toUbx()
        return logData

    def ledStatus(self):
        if self._SCREEN == self._OFFSCREEN:
            return

        self._leds.blink(self._LOGGER_LED,
                         GREEN if self._gps.signalOk else RED)

        if DEBUG:
            self._leds.blink(
                1, GREEN if self._gps._validationResults["pdop"] < 4 else RED)
            self._leds.blink(
                2, GREEN if self._gps._validationResults["validtime"] else RED)
            self._leds.blink(
                3, GREEN if self._gps._validationResults["fullyresolved"] else RED)
            self._leds.blink(
                4, GREEN if self._gps._validationResults["validllh"] else RED)

    def sendAllData(self):
        """Send All Data"""
        self._GPS_DATA_COUNTER_IN_SECONDS += 1

        if self._GPS_DATA_COUNTER_IN_SECONDS % self._LOG_INTERVAL_IN_SECONDS != 0:
            if DEBUG:
                display("Skipping Data because of _LOG_INTERVAL_IN_SECONDS")
            return

        self._sensors.read()
        self._accelerometer.read()

        # when the microphone is not read in the main loop we need to read it here
        if self._logger.enabled:
            if self._gps._signalOk:
                if self._gps._changeFileName and self._gps._justStarted:
                    self._logger.changeLogFileName(self._gps.getTimeStamp())
                    self._gps._changeFileName = False
                    self._gps._justStarted = False
                self._logger.append(self.buildLogData())
            else:
                if DEBUG:
                    display("NOT LOGGING BECAUSE OF INVALID GPS DATA")

        if DEBUG:
            display(binascii.hexlify(self._gps.toUbx()))
            display(binascii.hexlify(self._accelerometer.toUbx()))
            display(binascii.hexlify(self._microphone.toUbx()))
            display(binascii.hexlify(self._sensors.toUbx()))

        if self._ble.enabled:
            if self._ble.connected:
                if JSON_TO_BLE:
                    json = {}
                    self._gps.toJson(json)
                    self._accelerometer.toJson(json)
                    self._microphone.toJson(json)
                    self._sensors.toJson(json)
                    self._ble.write("{}".format(json))
                    self._ble.write('\n')
                    if DEBUG:
                        display("{}".format(json))
                else:
                    logData = self._gps.toUbx()
                    logData += self.extendUbx(12+8+8)
                    logData += self._accelerometer.toUbx()
                    logData += self._microphone.toUbx()
                    logData += self._sensors.toUbx()
                    self._ble.write(self.buildLogData())
                self._leds.blink(self._LOGGER_LED, BLUE)

        self.ledStatus()

    def onBleConnectionStateChanged(self):
        if self._ble.connected:
            display("BLE Client connected")
        else:
            display("BLE Client disconnected")
            self._ble.startAdvertising()

    def onBleAdvertising(self):
        pass

    def onWrite(self, data):
        pass

    def onLogData(self, data, diskStatus):
        """
        if diskStatus == "Disk Write":
            self._leds.blink(9, GREEN)
        if diskStatus == "Disk Full":
            self._leds.blink(9, ORANGE)
        if diskStatus == "Disk Protected":
            self._leds.blink(9, RED)
        if diskStatus.startswith("New File"):
            self._leds.blink(9, PINK)
        """
        pass

    def AKeyPressed(self):
        # display("A")
        if self._SCREEN == self._GPS_SCREEN:
            if self._gps.enabled:
                self._speaker.playFile("off.wav")
                self._SCREEN = 10
                self._ble.stopAdvertising()
                self._gps.disable()
                self._ble.disable()
                self._sensors.disable()
                self._accelerometer.disable()
                self._microphone.disable()
                self._logger.disable()
                self._leds.set(self._LOGGER_LED, BLACK)
            else:
                self._speaker.playFile("on.wav")
                self._SCREEN = 0
                self._ble.startAdvertising()
                self._gps.enable()
                self._ble.enable()
                self._sensors.enable()
                self._accelerometer.enable()
                self._microphone.enable()
                self._logger.enable()
                self._leds.set(self._LOGGER_LED, BLUE)
                if self._logger._diskFull:
                    self._leds.set(self._LOGGER_LED, ORANGE)
                if not self._logger._writable:
                    self._leds.set(self._LOGGER_LED, RED)

        if self._SCREEN == self._BLE_SCREEN:
            if self._ble.isAdvertising:
                self._ble.stopAdvertising()
                self._leds.set(self._BLE_LED, BLACK)
            else:
                self._ble.startAdvertising()
                self._leds.set(self._BLE_LED, BLUE)

        if self._SCREEN == self._MIC_SCREEN:
            if self._microphone.enabled:
                self._microphone.disable()
                self._leds.set(self._MIC_LED, BLACK)
            else:
                self._microphone.enable()
                self._leds.set(self._MIC_LED, GREEN)

        if self._SCREEN == self._DISK_SCREEN:
            self._logger.deleteLogs()

    def BKeyPressed(self):
        # display("B")
        if self._SCREEN != self._OFFSCREEN:
            self._leds.clear(self._SCREEN)

        self._SCREEN += 1

        if self._SCREEN > self._OFFSCREEN:
            self._SCREEN = 0

        if self._SCREEN != self._OFFSCREEN:
            self._leds.set(self._SCREEN, CYAN)

    def checkFreeMem(self):
        freeMem = gc.mem_free()
        display("{:.2f} kb free".format(freeMem/1024))
        return freeMem

    def mainLoop(self):
        self._buttons.read()

        if self._microphone.enabled:
            self._microphone.read()
            if self._SCREEN != self._OFFSCREEN:
                self._leds.showPeak(
                    self._MIC_LED, self._microphone._level, 80, 150)

        if self._gps.enabled:
            self._gps.read()

        self._buttons.read()

        self._ble.read()


class PixeltrackerApp():
    def __init__(self):
        self._pixeltracker = Pixeltracker()

    def run(self):
        if DEBUG:
            print(os.listdir())
        while True:
            self._pixeltracker.mainLoop()

# Main Loop


def main():
    ON_BAT = not supervisor.runtime.serial_connected
    boardLED = digitalio.DigitalInOut(board.D13)
    boardLED.switch_to_output()
    pixeltrackerApp = PixeltrackerApp()
    pixeltrackerApp.run()
    """
    try:            
        pixeltrackerApp.run()
    except Exception as e:
        boardLED.value = True
        display(e)
        time.sleep(1)
        boardLED.value = False
        time.sleep(1)
    """


if __name__ == '__main__':
    main()
