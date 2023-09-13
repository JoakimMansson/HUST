import serial
import re
import win32com.client


port_name = "COM10"

encodings = {
            #BMS_Pack
            10: "PackCurrent",
            11: "PackVoltage",
            12: "PackStateOfHealth",
            13: "AvgPackCurrent",
            14: "PackStateOfHealth",
            #BMS_Cell
            15: "LowCellVoltage",
            16: "HighCellVoltage",
            17: "AvgCellVoltage",
            #BMS_Failsafes
            18: "VoltageFailsafeActive",
            19: "CurrentFailsafeActive",
            20: "RelayFailsafeActive",
            21: "CellBalancingActive",
            22: "ChangeinterlockFailsafeActive",
            23: "ThermistorB_valueTableInvalid",
            24: "InputPoweruSupplyFailed",
            #BMS_Temperatures
            25: "HighestTemperature",
            26: "LowestTemperature",
            27: "AverageTemperature",
            28: "InternalTemperature",

            #MC_Temperatures
            29: "HeatsinkTemperature",
            30: "MotorTemperature",
            #MC_CurrentVoltage
            31: "BusCurrent",
            32: "BusVoltage",
            #MC_Velocity
            33: "MotorVelocity",
            34: "VehicleVelocity",
            #MC_ErrorFlags
            35: "MotorOverSpeed",
            36: "DesaturationFault",
            37: "RailUnderVoltage",
            38: "ConfigReadError",
            39: "WatchdogCausedLastReset",
            40: "BadMotorPositionHallSequence",
            41: "DCBusOverVoltage",
            42: "SoftwareOverCurrent",
            43: "HardwareOverCurrent",
            #MC_LimitFlags
            44: "IPMTemperatureOrMotorTemperature",
            45: "BusVoltageLowerLimit",
            46: "BusVoltageUpperLimit",
            47: "BusCurrent",
            48: "Velocity",
            49: "MotorCurrent",
            50: "OutputVoltagePWM"
        }

ser = serial.Serial(
    port = port_name,
    baudrate=9600
)

def getAllData():
    data = str(ser.read_all())
    return data

def getRawData():
    data = str(ser.read_all())

    if not contains("recv", data) or not contains("natl", data):
        return None


    index = 0
    counter = 0
    for char in data:
        if char == "r" and counter == 0:
            counter = 1
        elif char == "r" and counter == 1:
            index = index - 2
            break
        
        index = index + 1

    finalData = ""
    print(data)
    while data[index] != ",":
        finalData = data[index] + finalData
        index = index - 1
    
    return finalData

def contains(string: str, txt: str):
    if re.search(string, txt) == None:
        return False
    else:
        return True


while(True):
    #data = getRawData()
    data = str(ser.read_all())
    if data != "b''":
        print(data)
   # if data != None:
        #print(data)
    
    
    #data = getRawData()
    #if data == None:
    #    pass
    #else:
    #    print(data)

