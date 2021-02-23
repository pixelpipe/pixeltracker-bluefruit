"""
Jupyter Example:
        import pandas
        import matplotlib.pyplot as plt
        from matplotlib.pyplot import figure
        from pandas import DataFrame, Series
        from ubxparser import UbxParser
        parser = UbxParser()
        data = []
        for i in range(3):
            data += parser.read(f"C:\\projects\\pixeltracker\\utils\\ubxparser\\tests\\{i}.pubx", AccurateDataOnly = True)
        gps = DataFrame.from_dict(data)
        gps.plot(kind='line',y='Light')        

Orange Example: 
        import sys
        import struct
        import binascii
        import pandas as pd
        from pandas import DataFrame
        from Orange.data.pandas_compat import table_from_frame
        pd.set_option("display.max_columns", None)
        pd.set_option("precision", 4)
        import sys
        if not "C:\\projects\\pixeltracker\\utils\\ubxparser\\ubxparser" in sys.path:
            sys.path.append("C:\\projects\\pixeltracker\\utils\\ubxparser\\ubxparser")
        from ubxparser import UbxParser
        parser = UbxParser()
        data = []
        for i in range(3):
            data += parser.read(f"C:\\projects\\pixeltracker\\utils\\ubxparser\\tests\\{i}.pubx", AccurateDataOnly = True)
        outFrame = DataFrame.from_dict(data)
        out_data = table_from_frame(outFrame)
"""
import struct
import binascii

_UBX_FORMAT = {
    (0x01, 0x07) : "<LHBBBBBBLLBBBBllllLLlllllLLHBBBBBBlHHBBHfffffff"
    }

_SEPARATOR = '\t'
_NEWLINE = '\n'

class UbxParser():
    def __init__(self):
        pass

    def __del__(self):
        pass

    def _readBytes(self, stream, count):
        try:
            bytes = stream.read(count)
            if len(bytes)<count:
                return None
        except:
            print(f"Error reading file")
            return None
        return bytes


    def read(self, filename) -> []:
        gotUbx = False
        lastKeyValue = 0
        json = {}
        data = []
        gotGps = False
        try:               
            stream = open(filename, "rb")
        except Exception as err:
            print(f"File not found {err}")
            return []
        while True:
            magic1 = self._readBytes(stream,1)
            if magic1 == None:
                break
            if magic1 == b"\xb5":
                magic2 = self._readBytes(stream,1)
                if magic2 == None:
                    break
                if magic2 == b"\x62":
                    gotUbx = True
            if gotUbx:
                headerChunk = self._readBytes(stream,4)
                if headerChunk == None:
                    break
                header = struct.unpack("<BBH",headerChunk)
                classId = header[0]
                msgId = header[1]
                payloadLen = header[2] + 4 + 12 + 8 + 8
                payloadChunk = self._readBytes(stream,payloadLen)
                if payloadChunk == None:
                    break
                payloadFormat = _UBX_FORMAT[(classId,msgId)]
                if payloadFormat is None:
                    print(f"Unknown UBX block : {classId:02x} {msgId:02x}")
                    continue
                payloadCalcSize = struct.calcsize(payloadFormat)
                if payloadCalcSize != payloadLen:
                    print(f"Wrong length for {classId:02x} {msgId:02x}. Expected {payloadLen}, calculated {payloadCalcSize}")
                else:
                    X = struct.unpack(payloadFormat, payloadChunk)
                    if classId == 0x01 and msgId == 0x07:
                        date_time_str = f'{X[3]:02d}/{X[2]:02d}/{X[1]} {X[4]:02d}:{X[5]:02d}:{X[6]:02d}'
                        json["Time"] = date_time_str
                        json["SatCount"] = X[13]
                        json["Longitude"] = X[14] / 10000000
                        json["Latitude"] = X[15] / 10000000
                        json["Height"] = X[16] / 1000
                        json["HMSL"] = X[17]
                        json["HorAcc"] = X[18] / 1000
                        json["VertAcc"] = X[19] / 1000
                        json["ITOW"] = X[0]
                        json["Year"] = X[1]
                        json["Month"] = X[2]
                        json["Day"] = X[3]
                        json["Hour"] = X[4]
                        json["Min"] = X[5]
                        json["Sec"] = X[6]
                        json["Valid"] = X[7]
                        json["TimeAcc"] = X[8]
                        json["FractionOfSecond"] = X[9]
                        json["FixType"] = X[10]
                        json["FixStatusFlags"] = X[11]
                        json["AdditionalFlags2"] = X[12]
                        json["NEDVelocityNorth"] = X[20] * 3.6 / 1000
                        json["NEDVelocityEast"] = X[21] * 3.6 / 1000
                        json["NEDVelocityDown"] = X[22] * 3.6 / 1000
                        json["GroundSpeed"] = X[23] * 3.6 / 1000,
                        json["HeadingOfMotion"] = X[24]
                        json["SpeedAccuracy"] = X[25] * 3.6 / 1000
                        json["HeadAccuracy"] = X[26]
                        json["PositionDOP"] = X[27]
                        json["AdditionalFlags3"] = X[28]
                        json["Reserved1"] = X[29]
                        json["HeadingOfVehicle"] = X[30]
                        json["MagneticDeclination"] = X[31]
                        json["MagneticDeclinationAccuracy"] = X[22]
                        self.parseValidationFlags(json, X[7], X[11])
                        #Extender UBX Data
                        json["ExE"] = X[32]
                        json["ExV"] = X[33]
                        json["ExLen"] = X[34]
                        json["GravityX"] = X[35]
                        json["GravityY"] = X[36]
                        json["GravityZ"] = X[37]
                        json["MicNoise"] = X[38]
                        json["MicLevel"] = X[39]
                        json["Temperature"] = X[40]
                        json["Light"] = X[41]
                        # fix date
                        #date_time_str = '18/09/19 01:55:19'
                        #date_time_str = f"18/09/19 01:55:19"
                        #date_time_obj = datetime.strptime(date_time_str, '%d/%m/%y %H:%M:%S')
                        #print "The type of the date is now",  type(date_time_obj)
                        #print "The date is", date_time_obj
                        gotGps = True
                    if gotGps:
                        """
                        if AccurateDataOnly:
                            accurate = json["HorAcc"] < 100 and json["VertAcc"] < 100                        
                            if json["FullyResolved"] and accurate:                                
                                data.append(json)
                        else:
                            data.append(json)
                        """
                        data.append(json)
                        gotGps = False
                        json = {}

                    #print(json)
                gotUbx = False
        stream.close()
        return data

    def parseValidationFlags(self, jsonSource, validFlags, fixFlags):
            # print("{0:b}".format(flags))
            if validFlags & 0b00001000:
                jsonSource['ValidMag'] = True
            else:
                jsonSource['ValidMag'] = False

            if validFlags & 0b00000100:
                jsonSource['FullyResolved'] = True
            else:
                jsonSource['FullyResolved'] = False

            if validFlags & 0b00000010:
                jsonSource['TimeResolved'] = True
            else:
                jsonSource['TimeResolved'] = False

            if validFlags & 0b00000001:
                jsonSource['DateResolved'] = True
            else:
                jsonSource['DateResolved'] = False

            if fixFlags & 0b00100000:
                jsonSource['HedingOfVehicleValid'] = True
            else:
                jsonSource['HedingOfVehicleValid'] = False

            if fixFlags & 0b00010000:
                jsonSource['PSMState'] = True
            else:
                jsonSource['PSMState'] = False

            if fixFlags & 0b00000010:
                jsonSource['DiffSol'] = True
            else:
                jsonSource['DiffSol'] = False

            if fixFlags & 0b00000001:
                jsonSource['GNSSFixOk'] = True
            else:
                jsonSource['GNSSFixOk'] = False

    def toCsv(self, data):
        """Converts an array of dictionaries into CSV"""
        first = True
        str = ""
        for json in data:        
            if first:
                # Add the Header
                keys = json.keys()
                for key in keys:
                    str += key + _SEPARATOR
                str = str[:-1]
                str += _NEWLINE
                first = False
            else:
                values = json.values()
                for value in values:
                    str += "{}".format(value) + _SEPARATOR
                str = str[:-1]
                str += _NEWLINE
        return str

if __name__ == "__main__":
    import sys
    parser = UbxParser()
    #packets = parser.read(sys.argv[1])
    packets = parser.read("C:\\projects\\pixeltracker\\utils\\ubxparser\\ubxparser\\0.pubx")
    csv = parser.toCsv(packets)
    print(csv)

    #Pandas example
    #gps = DataFrame.from_dict(parser.GpsData)
    #print(gps)
    #acc = DataFrame.from_dict(parser.AccelerometerData)
    #print(acc)
    #sensors = DataFrame.from_records(parser.SensorsData)
    #print(sensors)
    #mic = DataFrame.from_dict(parser.MicrophoneData)
    #print(mic)    
    #parser.parse()
