from var import SEP, LOG_USING_UBX_FORMAT
import digitalio
import busio
import time
import struct
import board
import binascii
import math
from utils import Timer, COLORS

class Gps():
    """Handles the GPS data"""        

    def __init__(self, onDataHandler, activity,  gmtOffset = 1, uartReadBufferSize = 1, baudRate = 9600):
        self._activity = activity
        self._onDataHandler = onDataHandler
        self._GMT_OFFSET = gmtOffset
        self._UART_READ_COUNT = 2 #uartReadBufferSize
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
        self.DATA_ERROR = ""
        self.UBX_OUT = b""

        self.nav_pvt = ()
        """
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

        self.YEAR = 0
        self.MONTH = 0        
        self.DAY = 0
        self.HOUR = 0
        self.MIN = 0
        self.SEC = 0
        self.MS = 0
        """
        
        self.GPS_TIME = ""

        self._NAV_PVT = {
            "ON"    :   "B5 62 06 01 08 00 01 07 00 01 00 00 00 00 18 E1",
            "ON1"   :   "B5 62 06 01 08 00 01 07 01 01 00 01 01 00 1B EC",
            "OFF"   :   "B5 62 06 01 08 00 01 07 00 00 00 00 00 00 17 DC",
            "M1"    :   "B5 62 05 01 02 00 06 01 0F 38",
            "M2"    :   "B5 62 0A 04 00 00 0E 34"
            }
        self._NAV_STATUS = {
            "ON"    :   "B5 62 06 01 08 00 01 03 00 01 00 00 00 00 14 C5",
            "OFF"   :   "B5 62 06 01 08 00 01 03 00 00 00 00 00 00 13 C0",
            "CFG"   :   "B5 62 06 01 02 00 01 03 0D 36",
            "ACK"   :   "B5 62 05 01 02 00 06 01 0F 38"
            }
        self._POSLLH = {
            "ON"    :   "B5 62 06 01 08 00 01 02 00 01 00 00 00 00 13 BE",
            "OFF"   :   "B5 62 06 01 08 00 01 02 00 00 00 00 00 00 12 B9",
            "CFG"   :   "B5 62 06 01 02 00 01 02 0C 35",
            "ACK"   :   "B5 62 05 01 02 00 06 01 0F 38"
        }

        self._STORE  =  {
            "CFG"   :   "B5 62 06 09 0D 00 00 00 00 00 02 00 00 00 00 00 00 00 03 21 CE",
            "ACK"   :   "B5 62 05 01 02 00 06 09 17 40"
        }

        self._NMEA = {
            "GSA"    : "B5 62 06 01 08 00 F0 02 00 01 00 00 00 00 02 36",
            "xSAOFF" : "B5 62 06 01 08 00 F0 02 00 00 00 00 00 00 01 31",
            "xSVOFF" : "B5 62 06 01 08 00 F0 03 00 00 00 00 00 00 02 38",
            "xLLOFF" : "B5 62 06 01 08 00 F0 01 00 00 00 00 00 00 00 2A ",
            "xGAOFF" : "B5 62 06 01 08 00 F0 00 00 00 00 00 00 00 FF 23",
            "xTGOFF" : "B5 62 06 01 08 00 F0 05 00 00 00 00 00 00 04 46",
            "xMCOFF" : "B5 62 06 01 08 00 F0 04 00 00 00 00 00 00 03 3F",
            "ACK"    :  "B5 62 05 01 02 00 06 01 0F 38"
        }

        self._CFG   =   {
            "DEFAULT"   : "B5 62 06 09 0D 00 FF FF 00 00 00 00 00 00 FF FF 00 00 03 1B 9A",
            "REVERT"    : "B5 62 06 09 0D 00 00 00 00 00 00 00 00 00 FF FF 00 00 03 1D B3",
            "ACK"       : "B5 62 05 01 02 00 06 09 17 40"
        }

    def send(self, hexString):
        data = binascii.unhexlify(hexString.replace(' ',''))
        for i in range(len(data)):
            print("{:02X} ".format(data[i]), end='')
        self._uart.write(data)
        print("")
        # time.sleep(0.5)

    def config(self):
        self.enable()
        time.sleep(0.1)
        self.send(self._NAV_PVT["ON"])
        #self.send(self._NAV_PVT["M1"])
        #self.send(self._NAV_PVT["M2"])
        #self.send(self._NAV_PVT["CFG"])
        #self.send(self._NAV_PVT["ACK"])

        #self.send(self._NAV_STATUS["ON"])        
        #self.send(self._NAV_STATUS["CFG"])
        #self.send(self._NAV_STATUS["ACK"])

        #self.send(self._POSLLH["ON"])
        #self.send(self._POSLLH["CFG"])
        #self.send(self._POSLLH["ACK"])

        #self.send(self._NMEA["GSA"])
        """
        self.send(self._NMEA["xSAOFF"])
        self.send(self._NMEA["xSVOFF"])
        self.send(self._NMEA["xLLOFF"])
        self.send(self._NMEA["xGAOFF"])
        self.send(self._NMEA["xTGOFF"])
        self.send(self._NMEA["xMCOFF"])
        """
        #self.send(self._NMEA["ACK"])
        #self.send(self._CFG["REVERT"])
        #self.send(self._CFG["ACK"])

        self.send(self._STORE["CFG"])
        #self.send(self._STORE["ACK"])

        #self.disable()
        time.sleep(1)

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

    def parseToUbx(self, block):
        if len(block)<=3:
            self.DATA_ERROR = "Not UBX" # No UBX
            return False

        # Extractinng UBX Class, ID and Len
        self.UBX_CLASS = block[0]
        self.UBX_ID = block[1]
        self.UBX_LEN = block[2] + block[3] * 256
        self.UBX_OUT = block
        return True        

    def parseToString(self,block):
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
            if self.UBX_ID==0x07:
                size=struct.calcsize('<LHBBBBBBLLBBBBllllLLlllllLLHBBBBBBlhL')
                #print(size+4)
                #print(len(block))
                if not ((size + 4) == len(block)):
                    self.DATA_ERROR = "Broken UBX[{}/{}]".format(size + 4, len(block)) # incomplete UBX
                    return False
                self.nav_pvt = struct.unpack_from('<LHBBBBBBLLBBBBllllLLlllllLLHBBBBBBlhL',block,4)
                self.ITOW = self.nav_pvt[0]
                self.YEAR = self.nav_pvt[1]
                self.MONTH = self.nav_pvt[2]
                self.DAY = self.nav_pvt[3]
                self.HOUR = self.nav_pvt[4]
                self.MIN = self.nav_pvt[5]
                self.SEC = self.nav_pvt[6]
                self.VALID = self.nav_pvt[7]
                self.DATA_ERROR = ""

                if self.VALID & 0b00001000:
                    self.VALID_MAG = True
                    self.DATA_ERROR += "M"
                else:
                    self.VALID_MAG = False
                    self.DATA_ERROR += ""

                if self.VALID & 0b00000100:
                    self.FULLY_RESOLVED = True
                    self.DATA_ERROR += "R"
                else:
                    self.FULLY_RESOLVED = True
                    self.DATA_ERROR += ""

                if self.VALID & 0b00000010:
                    self.VALID_TIME = True
                    self.DATA_ERROR += "T"
                else:
                    self.VALID_TIME = False
                    self.DATA_ERROR += ""

                if self.VALID & 0b00000001:
                    self.VALID_DATE = True
                    self.DATA_ERROR += "D"
                else:
                    self.VALID_DATE = False
                    self.DATA_ERROR += ""

                self.TACC = self.nav_pvt[8]
                self.NANO = self.nav_pvt[9]
                self.FIXTYPE = self.nav_pvt[10]
                self.FLAGS = self.nav_pvt[11]
                self.FLAGS2 = self.nav_pvt[12]
                self.NUMSV = self.nav_pvt[13]
                self.LON = self.nav_pvt[14]
                self.LAT = self.nav_pvt[15]
                self.ALT = self.nav_pvt[16]
                self.HMSL = self.nav_pvt[17]
                self.HACC = self.nav_pvt[18]
                self.VACC = self.nav_pvt[19]
                self.VELN = self.nav_pvt[20]
                self.VELE = self.nav_pvt[21]
                self.VELD = self.nav_pvt[22]
                self.SPEED = self.nav_pvt[23]
                self.HEADMOT = self.nav_pvt[24]
                self.SACC = self.nav_pvt[25]
                self.HEADACC = self.nav_pvt[26]
                self.PDOP = self.nav_pvt[27]
                self.FLAGS3 = self.nav_pvt[28]
                self.RSVD1 = self.nav_pvt[29]
                self.HEADVEH = self.nav_pvt[30]
                self.MAGDEC = self.nav_pvt[31]
                self.MAGACC = self.nav_pvt[32]
                # print(self.nav_pvt)                
                return True

        print("ubx 0x{:02x} 0x{:02x} {}".format(self.UBX_CLASS,self.UBX_ID,self.UBX_LEN))
        self.DATA_ERROR = "Unknown UBX"
        return False

    def toggle(self):
        if self.isEnabled:
            self.disable()
        else:
            self.enable()
    """
    def read(self):        
        while True:
            data = self._uart.read(1)
            if data is None:
                break
            if data[0] == 0xb5:
                break
        while True:
            data = self._uart.read(1)
            if data is None:
                break
            if data[0] == 0x62:
                break
        print("Bingo!")
    """

    #@timed_function
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
            for i in range(len(data)):
                # check zero buffer
                if len(data)<=0:
                    return
                # get the byte
                self._CRT_UART_BYTE=data[i]
                """
                if self._CRT_UART_BYTE==0x0d:
                    # ascii
                    print(self._CURRENT_UART_BLOCK.decode('ascii'),end="")
                    self._CURRENT_UART_BLOCK=bytearray()
                    """
                # if separator
                if self._PREV_UART_BYTE==0xb5 and self._CRT_UART_BYTE==0x62:
                    # Bingo! Process previous block
                    if LOG_USING_UBX_FORMAT:
                        self.validData = self.parseToUbx(self._CURRENT_UART_BLOCK[1:-1])
                    else:
                        self.validData = self.parseToString(self._CURRENT_UART_BLOCK[1:-1])
                    if self._activity:
                        if self.validData:
                            if (self.DATA_ERROR == "RTD"):
                                #self.NUMSV = 10
                                if self.NUMSV>0:
                                    #self._activity.setColor((0, 10*self.NUMSV,0))
                                    self._activity.setColor(COLORS["green"])
                                else:
                                    self._activity.setColor(COLORS["cyan"])
                            else:
                                if (self.DATA_ERROR == ""):
                                    self._activity.warning()
                                else:
                                    self._activity.enable()
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
        GPS_TIME = '{}{}{:02d}:{:02d}:{:02d}'.format(daystr, SEP, self._GMT_OFFSET + HOUR, MIN, int(MS/1000))
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
        str = ""
        str += "{:.6f}".format(self.LAT/10000000)+SEP
        str += "{:.6f}".format(self.LON/10000000)+SEP
        str += "{}".format(self.ALT)+SEP
        str += "{}".format(self.NUMSV)+SEP
        str += "{},{},{}".format(self.YEAR, self.MONTH, self.itow2str(self.ITOW))+SEP
        str += "{}".format(self.HACC)+SEP
        str += "{}".format(self.VACC)+SEP
        str += "{}".format(self.SPEED)+SEP
        str += "{}".format(self.HEADMOT)+SEP
        str += "{}".format(self.HEADACC)+SEP
        str += "{}".format(self.HEADVEH)+SEP        
        str += "{}".format("" if self.DATA_ERROR == "" else "["+self.DATA_ERROR+"]")
        return str

    def ubx(self):
        return self.UBX_OUT
