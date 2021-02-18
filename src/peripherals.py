import board
import struct
import math
import binascii
import time
import array

#import audiobusio

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

# Board LEDs
import digitalio

boardLED = digitalio.DigitalInOut(board.D13)
boardLED.switch_to_output()

def boardLedOn():
    boardLED.value = True

def boardLedOff():
    boardLED.value = True

def boardLedToggle():
    boardLED.value = not boardLED.value

"""
import alarm
def sleepAndWakeAfter(seconds):
    # Set an alarm for 60 seconds from now.
    time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + seconds)
    # Deep sleep until the alarm goes off. Then restart the program.
    alarm.exit_and_deep_sleep_until_alarms(time_alarm)

def storeIntoSleepMemory(slotId, data):
    alarm.sleep_memory[slotId] = data
"""

# Classes
import neopixel
class Pixels():
    """Handle the Neopixels"""
    def __init__(self):
        self._NUM_PIXELS = 10
        self.BRIGHTNESS = 0.01
        self._pixels = neopixel.NeoPixel(board.NEOPIXEL, self._NUM_PIXELS, brightness=self.BRIGHTNESS, auto_write=False)
        self._pixels.fill(0)
        self._pixels.show()

    def set(self, id, color):
        """Assign color to pixel"""
        self._pixels[id] = color
        self.show()

    def clear(self):
        self._pixels.fill(0)

    def show(self):
        self._pixels.show()

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
            #print("A ON")
            if not self._buttonAState:
                self._onButtonAPressed()
                self._buttonAState = True
        else:
            self._buttonAState = False
        if self._buttonB.value:
            #print("B On")
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

    def bothPressed(self):
        return self._buttonA.value and self._buttonB.value

import busio
import adafruit_lis3dh

class Accelerometer():
    """Accelerometer"""

    def __init__(self):
        # Configure the Accelerometer
        if hasattr(board, "ACCELEROMETER_SCL"):
            self._acc_i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
            self._lis3dh = adafruit_lis3dh.LIS3DH_I2C(self._acc_i2c, address=0x19)
        else:
            self._acc_i2c = busio.I2C(board.SCL, board.SDA)
            self._lis3dh = adafruit_lis3dh.LIS3DH_I2C(self._acc_i2c)
        # Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
        self._lis3dh.range = adafruit_lis3dh.RANGE_8_G
        self._lis3dh.set_tap(2,60)

    def read(self):
        self.data = [ value / adafruit_lis3dh.STANDARD_GRAVITY for value in self._lis3dh.acceleration]

    def tapped(self):
        return self._lis3dh.tapped

    def toString(self):
        return "{:+.4f}\t{:+.4f}\t{:+.4f}".format(self.data[0],self.data[1],self.data[2])

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

class Ble():
    """Handles BLE connections and data"""
    def __init__(self, activity):
        # Configure the BLE and the UART Server
        self._ble = BLERadio()
        self._activity = activity
        self._uart_server = UARTService()
        self._advertisement = ProvideServicesAdvertisement(self._uart_server)
        self._enabled = False

    def write(self, data):
        if self._ble.connected:
            self._uart_server.write(data)
            if self._activity:
                self._activity.ok()
        if self._activity:
            self._activity.off()

    def enable(self):
        if not self._ble.connected:
            self._ble.stop_advertising()
            self._ble.start_advertising(self._advertisement)
            print("Start Phone App and connect")
            if self._activity:
                self._activity.enable()
        self._enabled = True

    def disable(self):
        if self._ble.connected:
            self._ble.stop_advertising()
        if self._activity:
            self._activity.off()
        self._enabled = False

    def toggle(self):
        if self._enabled:
            self.disable()
        else:
            self.enable()

    @property
    def uart(self):
        return self._uart_server

    @property
    def ble(self):
        return self._ble

    @property
    def advertisement(self):
        return self._advertisement

    @property
    def isConnected(self):
        return self._ble.connected

import adafruit_thermistor
import analogio
class Sensors():
    """Sensors Class"""
    def __init__(self):
        # Configure the Thermistor for reading the temperature
        self._thermistor = adafruit_thermistor.Thermistor(board.TEMPERATURE, 10000, 10000, 25, 3950)
        self._light = analogio.AnalogIn(board.LIGHT)

    def scale(self, value):
        """Scale the light sensor values from 0-65535 (AnalogIn range)
        to 0-50 (arbitrarily chosen to plot well with temperature)"""
        return  (value / 65535 * 50) * 10

    def read(self):
        self.data = [self._thermistor.temperature, self.scale(self._light.value)]

    def toString(self):
        return "{:.2f}\t{:.2f}".format(self.data[0], self.data[1])

    @property
    def thermistor(self):
        return self._thermistor

    @property
    def light(self):
        return self._light

class Gps():
    """Handles the GPS data"""
    _UART_READ_COUNT = 1
    _GMT_OFFSET = 1

    def __init__(self, onDataHandler, activity,  gmtOffset = 1, uartReadBufferSize = 1, baudRate = 9600):
        self._activity = activity
        self._onDataHandler = onDataHandler
        self._GMT_OFFSET = gmtOffset
        self._UART_READ_COUNT = uartReadBufferSize
        self._baudRate = baudRate
        # Configure the PPS Signal Pin
        # D6 = A1 = PPS (INPUT)
        self._pps = digitalio.DigitalInOut(board.D6)
        self._pps.direction = digitalio.Direction.INPUT
        # Configure the GPS Enable PIN
        # D9 = A2 = EN  (OUTPUT)
        self._gpsEnable = digitalio.DigitalInOut(board.D9)
        self._gpsEnable.direction = digitalio.Direction.OUTPUT
        # Local UART
        self._uart = busio.UART(board.TX, board.RX, baudrate=self._baudRate)
        self._testTime = Timer()
        self.validData = False

        self._CRT_UART_BYTE = 0x00
        self._PREV_UART_BYTE = 0x00
        self._CURRENT_UART_BLOCK = bytearray()
        self._GPS_CHAR_COUNTER = 0

        self.UBX_CLASS = 0
        self.UBX_ID =0
        self.UBX_LEN = 0
        self.ITOW = 0
        self.LON = 0
        self.LAT = 0
        self.ALT = 0
        self.HMSL = 0
        self.HACC = 0
        self.VACC = 0
        self.CHKSUM = 0
        self.PREV_LON = 0
        self.PREV_LAT = 0
        self.BEARING = 0
        self.DATA_ERROR = ""

        self.DAY = 0
        self.HOUR = 0
        self.MIN = 0
        self.SEC = 0
        self.MS = 0
        self.GPS_TIME = ""

    def enable(self):
        self._gpsEnable.value = True
        if self._activity:
            self._activity.enable()
        print("GPS Enabled!")

    def disable(self):
        self._gpsEnable.value = False
        print("GPS Disabled!")
        if self._activity:
            self._activity.off()

    def bearing(self, lon1, lat1, lon2, lat2):
        """Calculates bearing from previous location"""
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon)
        brng = math.degrees(math.atan2(y, x))
        #brng = math.atan2(y, x).toDeg();
        #if brng < 0: brng+= 360
        return int(brng)

    def parse(self,block):
        """Processor for the UBX blocks
        https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf"""

        if len(block)<=3:
            self.DATA_ERROR = "Not UBX" # No UBX
            return False

        # Extractinng UBX Class, ID and Len
        self.UBX_CLASS = block[0]
        self.UBX_ID = block[1]
        self.UBX_LEN = block[2] + block[3] * 256

        # 0x01 0x02 - Block
        if self.UBX_CLASS==0x01:
            if self.UBX_ID==0x02:
                self.PREV_LON = self.LON
                self.PREV_LAT = self.LAT
                # 11 11 1111 11111111 11112222 22222222 22222222 33333333 33333333 33334444 4444
                # 00 11 2233 44556677 88990011 22334455 66778899 00112233 44556677 88990011 2233
                # 01 02 1c00 287c0600 00000000 00000000 00000000 98bdffff ffffffff 009c84df 1783
                # cl id len  ITOW     LON(deg) LAT(deg) ALT(mm)  HMSL(mm) HACC(mm) VACC(mm) cksm
                size=struct.calcsize('<LllllLLH')
                if not ((size + 4) == len(block)):
                    self.DATA_ERROR = "Broken UBX" # incomplete UBX
                    return False
                nav_posllh=struct.unpack_from('<LllllLLH',block,4)
                self.ITOW = nav_posllh[0]
                self.LON = nav_posllh[1]
                self.LAT = nav_posllh[2]
                self.ALT = nav_posllh[3]
                self.HMSL = nav_posllh[4]
                self.HACC = nav_posllh[5]
                self.VACC = nav_posllh[6]
                self.CHKSUM = nav_posllh[7]
                self.BEARING = self.bearing(self.PREV_LON,self.PREV_LAT,self.LON,self.LAT)
                self.DATA_ERROR = "UBX OK"
                return True
        #print("ubx 0x{:02x} 0x{:02x} {}".format(self.UBX_CLASS,self.UBX_ID,self.UBX_LEN))
        #print(binascii.hexlify(block))
        self.DATA_ERROR = "Unknown UBX"
        return False

    def toggle(self):
        if self.isEnabled:
            self.disable()
        else:
            self.enable()

    def read(self):
        # read a chunk from UART
        # do not read if the GPS is disabled
        if not self._gpsEnable.value:
            return

        # self._testTime.mark()
        data = self._uart.read(self._UART_READ_COUNT)
        #self._testTime.show("gps: ")
        self._GPS_CHAR_COUNTER = self._GPS_CHAR_COUNTER + self._UART_READ_COUNT
        # Parse the current chunk
        if data is not None:
            for i in range(self._UART_READ_COUNT):
                # check zero buffer
                if len(data)<=0:
                    return
                # get the byte
                self._CRT_UART_BYTE=data[i]
                if self._PREV_UART_BYTE==0xb5 and self._CRT_UART_BYTE==0x62:
                    # Bingo! Process previous block
                    self.validData = self.parse(self._CURRENT_UART_BLOCK[1:-1])
                    if self._activity:
                        if self.validData:
                            self._activity.ok()
                        else:
                            self._activity.error()
                    self._activity.off()
                    if self._onDataHandler:
                        self._onDataHandler(self, self.validData)
                    # Reset the block for the next fill
                    self._CURRENT_UART_BLOCK=bytearray()
                # Append the byte
                self._CURRENT_UART_BLOCK.append(self._CRT_UART_BYTE)
                # Current will be the previous
                self._PREV_UART_BYTE=self._CRT_UART_BYTE

    def itow2str(self, iTOW):
        """Converts iTOW to readable time"""

        dow = ["SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"]
        MIN, MS = divmod(iTOW, 60000)
        HOUR, MIN = divmod(MIN, 60)
        DAY, HOUR = divmod(HOUR, 24)
        if DAY>=0 and DAY<=6:
            daystr = dow[DAY]
        else:
            daystr = str(DAY)
        #GPS_TIME = '{} {:02d}:{:02d}:{:04.1f}'.format(daystr, GMT_OFFSET + HOUR, MIN, MS/1000)
        #GPS_TIME = '{} {:02d}:{:02d}:{:02d}'.format(daystr, GMT_OFFSET + HOUR, MIN, int(MS/1000))
        GPS_TIME = '{}\t{:02d}:{:02d}:{:02d}'.format(daystr, self._GMT_OFFSET + HOUR, MIN, int(MS/1000))
        return GPS_TIME

    @property
    def pps(self):
        return self._pps

    @property
    def gpsEn(self):
        return self._gpsEnable

    @property
    def isEnabled(self):
        return self._gpsEnable.value

    @property
    def isDisabled(self):
        return not self._gpsEnable.value

    def toString(self):
        return "{}\t{:.6f}\t{:.6f}\t{:.2f}\t{}".format(self.itow2str(self.ITOW), self.LAT/10000000, self.LON/10000000, self.ALT/1000,"" if self.DATA_ERROR == "" else "["+self.DATA_ERROR+"]")

class Activity():
    """Handles LED activity from various devices"""
    def __init__(self, pixels, pixel, offColor, enableColor, okColor, warningColor, errorColor, sleep = 0):
        self._pixels = pixels
        self._pixel = pixel
        self._enableColor = enableColor
        self._okColor = okColor
        self._warningColor = warningColor
        self._errorColor = errorColor
        self._sleep = sleep
        self._offColor = offColor
        self._stateColor = self._offColor

    def update(self):
        if self._sleep != 0:
            time.sleep(self._sleep)

    def on(self, autooff = 0):
        self._pixels.set(self._pixel, self._stateColor)
        self.update()

    def off(self):
        self._pixels.set(self._pixel, self._offColor)
        self.update()

    def ok(self):
        self._stateColor = self._okColor
        self._pixels.set(self._pixel, self._stateColor)
        self.update()

    def warning(self):
        self._stateColor = self._warningColor
        self._pixels.set(self._pixel, self._stateColor)
        self.update()

    def error(self):
        self._stateColor = self._errorColor
        self._pixels.set(self._pixel, self._stateColor)
        self.update()

    def enable(self):
        self._stateColor = self._enableColor
        self._pixels.set(self._pixel, self._stateColor)
        self.update()

    def colorOf(self, level, rgb):
        self._stateColor = ( rgb[0] * level, rgb[1] * level, rgb[2] * level )
        self._pixels.set(self._pixel, self._stateColor)
        self.update()

import audiobusio
class Microphone():
    """ Samples data from the microphone and returns the noise
        magnitude and level"""
    _NUM_SAMPLES = 160
    _SCALE_EXPONENT = math.pow(10, 2 * -0.1) # curve is 2

    def __init__(self, activity, numberOfSamples = 64):
        self._activity = activity
        _NUM_SAMPLES = numberOfSamples
        self._mic = audiobusio.PDMIn(board.MICROPHONE_CLOCK, board.MICROPHONE_DATA,sample_rate=16000, bit_depth=16)
        # Record an initial sample to calibrate. Assume it's quiet when we start.
        self._samples = array.array('H', [0] * self._NUM_SAMPLES)
        self._mic.record(self._samples, len(self._samples))
        # Set lowest level to expect, plus a little.
        self._input_floor = self.normalized_rms(self._samples) + 10
        self._input_ceiling = self._input_floor + 500
        self.level = 0
        self.magnitude = 0

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
        self.magnitude = self.normalized_rms(self._samples)

        # Compute scaled logarithmic reading in the range 0 to NUM_PIXELS
        self.level = self.log_scale(self.constrain(self.magnitude, self._input_floor, self._input_ceiling),self._input_floor, self._input_ceiling, 0, 255)

        if self._activity:
            self._activity.colorOf(self.level, [ 1, 0, 0] )

    def toString(self):
        return "{:.2f}\t{:.2f}".format(self.magnitude, self.level)

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

import storage
class SimpleLogger:
    """Basic logging on the local storage"""
    def __init__(self, logs, logSize, activity):
        self._logFile = "log.txt"
        self._logs = logs
        self._crtLog = 0
        self._logSize = logSize
        self._activity = activity
        self._writable = True
        self._loggedBytes = 0

    def append(self, data):
        """Appends data to a file"""
        try:
            if self._writable:
                if self._loggedBytes > self._logSize:
                    self._crtLog = self._crtLog + 1
                    if self._crtLog >= self._logs:
                        self._crtLog = 0
                    self._logFile = "log_{}.txt".format(self._crtLog)
                    with open(self._logFile, 'w') as fp:
                        fp.write(data)
                        fp.flush()
                    self._loggedBytes = 0
                with open(self._logFile, 'a') as fp:
                    fp.write(data)
                    fp.flush()
                    self._loggedBytes = self._loggedBytes + len(data)
            else:
                self._activity.error()
                self._activity.off()
        except OSError as e:
            if e.args[0] == 28:  # If the file system is full...
                self._activity.warning()
            else:
                self._activity.error()
            self._writable = False

    def appendLine(self, data):
        """Appends a line of data to a file"""
        self.append(data+"\n")
