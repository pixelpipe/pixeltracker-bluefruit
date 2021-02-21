import struct
import binascii
from pandas import DataFrame

UBX_FORMAT = {
    (0x01, 0x07) : "<LHBBBBBBLLBBBBllllLLlllllLLHBBBBBBlHH",
    (ord('S'),ord('A')) : "<fff",
    (ord('S'),ord('M')) : "<ff",
    (ord('S'),ord('S')) : "<ff"
}

class UbxParser():
    def __init__(self, filename):
        self._filename = filename
        self._gpsData = []
        self._sensorData = []
        self._accelerometerData = []
        self._microphoneData = []

    @property
    def GpsData(self):
        return self._gpsData

    @property
    def SensorsData(self):
        return self._sensorData

    @property
    def MicrophoneData(self):
        return self._microphoneData

    @property
    def AccelerometerData(self):
        return self._accelerometerData

    def open(self):
        try:                        
            self._stream = open(self._filename, "rb")
            self._opened = True
        except Exception as err:
            print(f"File not found {err}")


    def parse(self):
        stream = self._stream
        gotUbx = False
        while True:
            magic1 = stream.read(1)
            if len(magic1) < 1:
                break
            if magic1 == b"\xb5":
                magic2 = stream.read(1)
                if len(magic2) < 1:
                    break
                if magic2 == b"\x62":
                    gotUbx = True
            if gotUbx:
                headerChunk = stream.read(4)
                if len(headerChunk) < 4:
                    break
                header = struct.unpack("<BBH",headerChunk)
                classId = header[0]
                msgId = header[1]
                payloadLen = header[2]
                payloadChunk = stream.read(payloadLen)
                if len(payloadChunk) < payloadLen:
                    break
                
                payloadFormat = UBX_FORMAT[(classId,msgId)]
                if payloadFormat is None:
                    print(f"Unknown UBX block : {classId:02x} {msgId:02x}")
                    continue
                payloadCalcSize = struct.calcsize(payloadFormat)
                if payloadCalcSize != payloadLen:
                    print(f"Wrong length for {classId:02x} {msgId:02x}. Expected {payloadLen}, calculated {payloadCalcSize}")
                else:
                    X = struct.unpack(payloadFormat, payloadChunk)
                    json = {}
                    if classId == 0x01 and msgId == 0x07:
                        json = {
                            "type":"ubxNavPvt",
                            "numsv":X[13],
                            "lon":X[14],"lat":X[15],"alt":X[16],"hmsl":X[17],"hacc":X[18],"vacc":X[19],
                            "itow":X[0],
                            "year":X[1],"month":X[2],"day":X[3],"hour":X[4],"min":X[5],"sec":X[6],
                            "valid":X[7],"tacc":X[8],"nano":X[9],"fixtype":X[10],"flags":X[11],"flags2":X[12],
                            "veln":X[20],"vele":X[21],"veld":X[22],"speed":X[23],
                            "headmot":X[24],"sacc":X[25],"headacc":X[26],"pdop":X[27],"flags3":X[28],"rsvd1":X[29],"headveh":X[30],
                            "magdec":X[31],"magacc":X[22]
                        }
                        self._gpsData.append(json)
                    if classId == ord('S') and msgId == ord('A'):
                        json = {
                            "type":"accelerometer",
                            "accx":X[0],
                            "accy":X[1],
                            "accz":X[2]
                        }
                        self._accelerometerData.append(json)
                    if classId == ord('S') and msgId == ord('M'):
                        json = {
                            "type":"microphone",
                            "magnitude":X[0],
                            "level":X[1]
                        }
                        self._microphoneData.append(json)
                    if classId == ord('S') and msgId == ord('S'):
                        json = {
                            "type":"sensors",
                            "temperature":X[0],
                            "light":X[1]
                        }                                
                        self._sensorData.append(json)
                    #print(json)

                gotUbx = False


"""
parser = UbxParser("C:\\projects\\pixeltracker\\utils\\0.pubx")
parser.open()
parser.parse()

gps = DataFrame.from_dict(parser.GpsData)
print(gps)

acc = DataFrame.from_dict(parser.AccelerometerData)
print(acc)
sensors = DataFrame.from_records(parser.SensorsData)
print(sensors)

mic = DataFrame.from_dict(parser.MicrophoneData)
print(mic)
"""