import sqlite3
import random
#import pandas as pd


class sqlConnection():

    def __init__(self) -> None:
        self.conn = sqlite3.connect("data")
        self.cursor = self.conn.cursor()



    # Declare name, datatype and constraints
    # EX:
    # CREATE TABLE tableName(
    # id INT PRIMARY KEY,
    # name VARCHAR(20)
    # );
    # ex: createTable("tableName", "id INTEGER PRIMARY KEY AUTOINCREMENT", "name VARCHAR(20)")
    def createTable(self, tableName: str, *argv: str) -> None:
        command = "CREATE TABLE " + tableName + "("
        
        allArgs = []
        for arg in argv:
            allArgs.append(arg)

        for i in range(len(allArgs)):
            if i == len(allArgs) - 1:
                command = command + allArgs[i] + ");"
                break
            
            command = command + " " + allArgs[i] + ","

        try:
            self.cursor.execute(command)
            self.conn.commit()
            print(self.cursor.fetchall())
        except Exception as e:
            print("Failed creating table: " + tableName + "\n" + str(e))


    def insertInto(self, tableName: str, columns: str, values: str) -> None:
        command = "INSERT INTO " +  tableName + " " + columns + " "
        command = command + "VALUES " + values + ";"
        self.cursor.execute(command)
        self.conn.commit()
        print(self.cursor.fetchall())


    def selectAll(self, tableName: str) -> None:
        command = "SELECT * FROM " + tableName
        self.cursor.execute(command)
        
        # Pretty print
        #print(pd.read_sql_query(command, self.conn))

        print(self.cursor.fetchall())
    
    
    def execCommmand(self, cmd: str):
        self.cursor.execute(cmd)

        obj = self.cursor.fetchall()
        if len(obj) == 0:
            self.conn.commit()
            return None
        else:
            #print(obj)
            return obj
    
    def getCursor(self) -> object:
        return self.cursor




if __name__ == "__main__":
    sql = sqlConnection()
    #sql.createTable("MC_LimitFlags", "TimeDate TIMESTAMP NULL DEFAULT CURRENT_TIMESTAMP PRIMARY KEY", "IPMTemperatureOrMotorTemperature BOOL", "BusVoltageLowerLimit BOOL", "BusVoltageUpperLimit BOOL", "BusCurrent BOOL", "Velocity BOOL", "MotorCurrent BOOL", "OutputVoltagePWM BOOL")
    #sql.insertInto("BMS_Cell", "(LowCellVoltage, HighCellVoltage, AvgCellVoltage)", "(1337, 1337, 1337)")
    encodings = {
            #BMS_Pack
            10: "PackCurrent",
            11: "PackVoltage",
            12: "PackStateOfHealth",
            13: "AvgPackCurrent",
            14: "PackStateOfCharge",
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
            24: "InputPowerSupplyFailed",
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
            47: "BusCurrentLimit",
            48: "Velocity",
            49: "MotorCurrent",
            50: "OutputVoltagePWM"
    }
    #for i in range(10 ,51):

    #sql.createTable("InputPowerSupplyFailed", "id INTEGER PRIMARY KEY AUTOINCREMENT" ,"TimeData TIMESTAMP DEFAULT (datetime('now', 'localtime'))","InputPowerSupplyFailed BOOL")
    #obj = sql.execCommmand("SELECT * FROM PackCurrent WHERE TimeData >= date('now', 'localtime') ORDER BY ID DESC")
    #print(obj)
        #sql.execCommmand("PRAGMA table_info(PackCurrent)")
    sql.execCommmand("INSERT INTO " + "MotorTemperature" + "(MotorTemperature)" + "VALUES (" + str(1337.1337) + ")")
        #pass
    #sql.execCommmand("DROP TABLE InputPowerSupplyFailed")
