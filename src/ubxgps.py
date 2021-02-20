import time
import board
import neopixel
import struct
import busio
import digitalio
import binascii
import utils

FLAG_LOG_ON = True

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
                self._onDataHandler(self._ubx)
            # init ubx with the next bytes
            self._ubx=self._buffer[magicIndex:]
        else:
            # append buffer to ubx
            self._ubx += bytes(self._buffer)

        # ToDo: the only problem is when 1st char of the buffer is 0xb5
        #       or last one is 0x62

    def toString(self):
        """Return human readable data"""
        return binascii.hexlify(self._ubx)

    def ubxBuffer(self):
        """Return current UBX Buffer"""
        return self._buffer

    def ubx(self):
        """Return the current UBX"""
        return self._ubx

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
    def __init__(self, autoOffTime = 30, onAdvertising = None, onWrite = None):
        self._ble = BLERadio()
        self._uart_server = UARTService()
        self._onAdvertising = onAdvertising
        self._onWrite = onWrite
        self._advertisement = ProvideServicesAdvertisement(self._uart_server)
        self._enabled = False
        self._onAdvertising = onAdvertising
        self._onWrite = onWrite
        self._autoOffTime = autoOffTime        
        self._autoOffTimer =Timer()

    def write(self, data):
        if self._ble.connected:
            self._uart_server.write(data)
            if self._onWrite:
                self._onWrite(data)

    def enable(self):
        if not self._ble.connected:
            self._ble.stop_advertising()
            self._ble.start_advertising(self._advertisement)
            print("Start Phone App and connect in 30 seconds")
            if self._onAdvertising:
                self._onAdvertising()
            self._enabled = True

    def autoOff(self):
        self._autoOffTimer.stopWatch(self._autoOffTime * 1000) # 30 sec

    def disable(self):
        #if self._ble.connected:
        self._ble.stop_advertising()
        self._enabled = False

    def read(self):
        # if expired and not connected then disable advertising
        if self._autoOffTimer.stopped():
            if not self._ble.connected:
                self.disable()

    @property
    def enabled(self):
        return self._enabled

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


class Timer():
    """Timer class"""
    def __init__(self):
        self._NOW = 0
        self._PREV = self._NOW
        self.mark()
        self.DELTA = 0
        self._stopped = False
        self._stopWatch = 0

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

import adafruit_thermistor
import analogio

class Sensors():
    """Sensors Class"""
    def __init__(self):
        # Configure the Thermistor for reading the temperature
        self._thermistor = adafruit_thermistor.Thermistor(board.TEMPERATURE, 10000, 10000, 25, 3950)
        self._light = analogio.AnalogIn(board.LIGHT)
        self._enabled = False

    def scale(self, value):
        """Scale the light sensor values from 0-65535 (AnalogIn range)
        to 0-50 (arbitrarily chosen to plot well with temperature)"""
        return  (value / 65535 * 50) * 10

    def read(self):
        self.data = [self._thermistor.temperature, self.scale(self._light.value)]

    def toString(self):
        str=""
        str+="{:.2f}".format(self.data[0])+SEP
        str+="{:.2f}".format(self.data[1])
        return str

    def ubx(self):                
        return struct.pack("<BBHff",ord('S'), ord('S'), 4, self.data[0], self.data[1])

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
        self._gps = UbxGps(self.OnGpsPacket, 4)
        self._leds = Leds(10, 0.05)
        self._ble = Ble(30, self.onBleAdvertising, self.onWrite)
        self._speaker = Speaker()
        self._buttons = Buttons(self.AKeyPressed, self.BKeyPressed)
        self._sensors = Sensors()

    def OnGpsPacket(self, data):
        """On incomming buffer"""        
        print("{} [{}]".format(data, len(data)))
        self._leds.set(5, (255, 0, 0))
        self._leds.clear(5)

        if self._ble.connected:
            self._ble.write(data)

    def logSensorData(self):
        self._sensors.read()
        self._ble.write(self._sensors.ubx())
        self._leds.set(6, (0, 155, 0))
        self._leds.clear(6)
        pass

    def onBleAdvertising(self):
        #self._leds.set(4, (0, 255, 255))
        pass

    def AKeyPressed(self):
        #print("A")
        if self._gps.enabled:
            self._speaker.playFile("off.wav")
            self._gps.disable()            
        else:
            self._speaker.playFile("on.wav")
            self._gps.enable()

    def BKeyPressed(self):        
        #print("B")
        if self._ble.enabled:
            self._speaker.playFile("off.wav")
            self._ble.disable()
            self._sensors.disable()         
        else:
            self._speaker.playFile("on.wav")
            self._ble.enable()
            self._sensors.enable()
            self._ble.autoOff()

    def onWrite(self, data):        
        self._leds.set(4, (0, 0, 155))
        self._leds.set(4, (0, 0, 0))                

    def run(self):
        blinkTimer = Timer()
        gpsBootTimer = Timer()
        autoOffTimer = Timer()
        sendSensorDataTimer = Timer()
        #self._gps.enable()
        #self._ble.enable()        
        self._speaker.playFile("on.wav")

        didGPSBooted = False

        gpsBootTimer.stopWatch(3000)
        blinkTimer.stopWatch(50)
        sendSensorDataTimer.stopWatch(300)

        while True:
            self._buttons.read()

            if gpsBootTimer.stopped():
                didGPSBooted =  True

            if didGPSBooted and self._gps.enabled:
                self._gps.read()

            if sendSensorDataTimer.stopped():
                if self._ble.connected:
                    self.logSensorData()
                sendSensorDataTimer.reset()

            if blinkTimer.stopped():
                if self._ble.enabled:
                    if not self._ble.connected:
                        self._leds.blink(4,(0,77,77))                
                #if self._gps.enabled and not self._ble.connected:
                #    self._leds.blink(5,(77,0,0))                    
                blinkTimer.reset()

            self._ble.read()
