import serial
from serial.tools import list_ports
import time
import re




class USB():

    def __init__(self, port=None, baud=9600) -> None:
        self.baud_rate = baud

        if port is None:
            self.port = str(self.find_active_serial_port())
        else:
            self.port = port
        
        print(self.port,type(self.port))

        self.ser = serial.Serial(self.port, baud)
    

    def get_used_port(self):
        return self.ser.port

    
    def get_available_port(self):
        ports = list(list_ports.comports())
        for port in ports:
            try:
                temp_ser = serial.Serial(port.device, self.baud_rate)
                temp_ser.close()
                return port.device
            except Exception as e:
                print("Port " + str(port.device) + " not available")

        return None
    
    
    def find_active_serial_port(self):
        # List available serial ports
        available_ports = serial.tools.list_ports.comports()

        # Iterate through available ports and try to open them
        for port_info in available_ports:
            port = serial.Serial(port_info.device)
            try:
                # Attempt to read data from the port
                data = port.read(1)  # You can change the number of bytes to read

                if data:
                    print(f"Active Serial Port: {port_info.device}")
                    return port_info.device
                else:
                    port.close()
            except serial.SerialException:
                # Error occurred while trying to open the port, continue to the next one
                continue

        print("No active serial port found.")
        return None
    

    def print_all_ports(self):
        ports = list(list_ports.comports())
        ports_str = ""
        for port in ports:
            ports_str += "[" + port.device + "]"

        if ports_str == "":
            print("None available ports")
        
        print(ports_str)
    
    # Remove everything from serial that is not a LETTER or DIGIT
    def get_filtered_serial(self):
        data = str(self.ser.readline().decode('utf-8'))
        data = re.sub(r'[^a-zA-Z0-9]', '', data)

        # Remove all non-digits, ex b'1029\r\r\n' -> 1029
        #filtered_data = ''.join(filter(lambda x: x.isdigit(), data))

        return data
    
    def get_serial_data(self):
        data = str(self.ser.readline().decode('utf-8'))
        return data
    
    def send_string(self, data: str):
        data_bytes = data.encode('utf-8')
        self.ser.write(data_bytes)

    

if __name__ == '__main__':
    reader = USB(baud=9600)
    # Wait for START signal from LoRa/dev/ttyACM0
    waiting_for_start = True
    while waiting_for_start:
        try:
            serial_data = reader.get_filtered_serial()
            print(serial_data)
            if serial_data == "START": waiting_for_start = False
            else: print("Waiting for START...")
        except Exception as e:
            print(e)
            continue
    
    while not waiting_for_start:
        serial_data = reader.get_filtered_serial()
        print(serial_data)

        # Necessary for try & except since
        # the serial read will include blank spaces.
        try:
            id_part, integer_part, decimal_digit = parse_hex_value(serial_data)
            print(f"ID: {id_part}, Integer: {integer_part},{decimal_digit}")
        except Exception as e:
            print(e)
            continue
        


