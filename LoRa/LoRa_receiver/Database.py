import sqlite3
import pandas as pd


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
        else:
            print(obj)
    
    def getCursor(self) -> object:
        return self.cursor




if __name__ == "__main__":
    sql = sqlConnection()
    #sql.createTable("MC_LimitFlags", "TimeDate TIMESTAMP NULL DEFAULT CURRENT_TIMESTAMP PRIMARY KEY", "IPMTemperatureOrMotorTemperature BOOL", "BusVoltageLowerLimit BOOL", "BusVoltageUpperLimit BOOL", "BusCurrent BOOL", "Velocity BOOL", "MotorCurrent BOOL", "OutputVoltagePWM BOOL")
    #sql.insertInto("BMS_Cell", "(LowCellVoltage, HighCellVoltage, AvgCellVoltage)", "(1337, 1337, 1337)")
    
    #sql.execCommmand("DELETE FROM BMS_Pack WHERE PackVoltage=1.0;")
    #sql.execCommmand("SELECT * FROM BMS_Cell;")
    sql.execCommmand("PRAGMA table_info(BMS_Failsafes)")
