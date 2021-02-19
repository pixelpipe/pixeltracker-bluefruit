import board
import struct
import math
import binascii
import time
import array
from utils import COLORS, _SEP

#import audiobusio

# Board LEDs
import digitalio

boardLED = digitalio.DigitalInOut(board.D13)
boardLED.switch_to_output()

def boardLedOn():
    """Turns D13 on"""
    boardLED.value = True

def boardLedOff():
    """Turns D13 off"""
    boardLED.value = True

def boardLedToggle():
    """Toggles D13"""
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
        #self._lis3dh.range = adafruit_lis3dh.RANGE_8_G
        self._lis3dh.set_tap(2,60)

    def read(self):
        self.data = [ value / adafruit_lis3dh.STANDARD_GRAVITY for value in self._lis3dh.acceleration]

    def tapped(self):
        return self._lis3dh.tapped

    def toString(self):
        str=""
        str+="{:+.4f}".format(self.data[0])+_SEP
        str+="{:+.4f}".format(self.data[1])+_SEP
        str+="{:+.4f}".format(self.data[2])
        return str

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

    @property
    def isEnabled(self):
        return self._enabled

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
        str=""
        str+="{:.2f}".format(self.data[0])+_SEP
        str+="{:.2f}".format(self.data[1])
        return str

    @property
    def thermistor(self):
        return self._thermistor

    @property
    def light(self):
        return self._light

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

    def setPixel(self,id,color):
        self._pixels.set(id, color)

    def setColor(self,color):
        self._pixels.set(self._pixel, color)

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
        str = ""
        str += "{:.2f}".format(self.magnitude/100)+_SEP
        str += "{:.2f}".format(self.level/100)
        return str

import storage
class SimpleLogger:
    """Basic logging on the local storage"""
    def __init__(self, header, logs, logSize, activity):
        self._logFile = "0.txt"
        self._logs = logs
        self._crtLog = 0
        self._logSize = logSize
        self._activity = activity
        self._writable = True
        self._loggedBytes = 0
        self.header = header

    def checkNextFile(self):
        # check if log file size exceeds the allowed max size
        if self._loggedBytes > self._logSize:
            # increment log number
            self._crtLog = self._crtLog + 1
            # circular logs
            if self._crtLog >= self._logs:
                self._crtLog = 0
            # build the filename
            self._logFile = "log_{}.txt".format(self._crtLog)
            # initialize the new file and write the header befor logging
            if self._writable:
                with open(self._logFile, 'w') as fp:
                    fp.write(self.header+"\n")
                    fp.flush()
                    self._loggedBytes = len(self.header) + 1
            else:
                #print("SimWrite {} [{}]".format(self._logFile, self._loggedBytes))
                #print(self.header)
                self._loggedBytes = len(self.header) + 1

    def writeData(self, data):
        # write data
        if self._writable:
            with open(self._logFile, 'a') as fp:
                fp.write(data)
                fp.flush()
                self._loggedBytes = self._loggedBytes + len(data)
        else:
            #print("SimAppend {} [{}]".format(self._logFile, self._loggedBytes))            
            #print(data)
            self._loggedBytes = self._loggedBytes + len(data)
    
    def append(self, data):
        """Appends data to a file"""
        try:
            # logs only if writable, otherwise sends to stdio
            self.checkNextFile()
            self.writeData(data)
        except OSError as e:
            # cant write -> not writable
            if e.args[0] == 28:  # If the file system is full...
                self._activity.warning()
            else:
                self._activity.error()
            self._writable = False

    def appendLine(self, data):
        """Appends a line of data to a file"""
        self.append(data+"\n")
