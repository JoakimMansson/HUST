import struct
import joblib
import pandas as pd


# BATTERY MANAGEMENT SYSTEM #
internal_temp = []
high_temp = []
low_temp = []
avg_temp = []

high_thermistor_id = []
low_thermistor_id = []

pack_current = []
pack_current_avg, nr_pack_current_samples = 0, 0
pack_abs_current = []
pack_open_voltage = []
pack_inst_voltage = []
pack_SOC = []
avg_current = []
low_cell_voltage = []
high_cell_voltage = []
avg_cell_voltage = []

current_limit = []

# MOTOR CONTROLLER #
vehicle_velocity = []
vehicle_velocity_avg, nr_vehicle_velocity_samples = 0, 0
motor_velocity = []
heatsink_temp = []
motor_temp = []

bus_current = []
bus_current_avg, nr_bus_current_samples = 0, 0
bus_voltage = []

is_start_odometer = True
start_distance = 0
odometer = []

receive_error_count = 0
transmit_error_count = 0


'''
internal_temp = np.append(internal_temp, line[7:9])
            high_temp = np.append(high_temp, line[9:11])
            low_temp = np.append(low_temp, line[11:13])
            avg_temp = np.append(avg_temp, line[13:15])
            failsafe_status = np.append(failsafe_status, line[15:17])
            high_thermistor_id = np.append(high_thermistor_id, line[17:19])
            low_thermistor_id = np.append(low_thermistor_id, line[19:21])
'''

def ieee754_to_decimal(hex_values, is_last_4=True):
    # Split the input hex string into individual hex values

    if is_last_4:
        hex = ''.join(reversed(hex_values[4:]))
    else:
        hex = ''.join(reversed(hex_values[:4]))

    # Convert the concatenated hex string to a 32-bit binary string
    binary_str = bin(int(hex, 16))[2:].zfill(32)

    # Convert the binary string to a floating-point decimal number
    decimal_value = struct.unpack('f', struct.pack('I', int(binary_str, 2)))[0]

    return decimal_value


with open("Data from 2023-10-17/Gunpoint_sträcka_Max1.txt", "r") as f:
    for line in f:
        id = line[1:4]

        if id == "601":
            # Process CAN ID 601 data
            pack_SOC.append(int(line[5:7], 16)*0.5)
            internal_temp.append(int(line[7:9], 16))
            high_temp.append(int(line[9:11], 16))
            low_temp.append(int(line[11:13], 16))
            avg_temp.append(int(line[13:15], 16))
            high_thermistor_id = int(line[15:17], 16)
            low_thermistor_id = int(line[17:19], 16)

        elif id == "602":
            # Process CAN ID 602 data
            #pack_current.append(struct.unpack('<h', bytes.fromhex(line[5:9]))[0] * 0.01)
            # Process CAN ID 602 data
            raw_pack_current = int(line[5:9], 16)
            if raw_pack_current & 0x8000:  # Check if the MSB is set (negative value)
                raw_pack_current = -(0x10000 - raw_pack_current)
            pack_current.append(raw_pack_current * 0.1)
            if raw_pack_current * 0.1 > 1: 
                nr_pack_current_samples += 1
                pack_current_avg += raw_pack_current * 0.1
            #pack_current.append((int(line[5:7], 16) + int(line[7:9], 16)) *0.1)
            pack_open_voltage.append(int(line[9:13], 16)*0.1)
            low_cell_voltage.append(int(line[13:17], 16)*0.0001)
            high_cell_voltage.append(int(line[17:21], 16)*0.0001)

        elif id == "603":
            avg_cell_voltage.append((int(line[5:9], 16))*0.0001)
            #avg_cell_voltage.append((int(line[5:7], 16) + int(line[7:9], 16))*0.0001)
            current_limit.append(int(line[9:12], 16)*0.1)
            pack_inst_voltage.append(int(line[13:17], 16)*0.1)
            pack_abs_current.append(struct.unpack('<h', bytes.fromhex(line[17:21]))[0] * 0.1)

        elif id == "402":
            data = [line[5:21][i:i+2] for i in range(0, len(line[5:21]), 2)]
            mc_current = ieee754_to_decimal(data, is_last_4=True)
            bus_current.append(mc_current)
            if mc_current > 1:
                nr_bus_current_samples += 1
                bus_current_avg += mc_current
            bus_voltage.append(ieee754_to_decimal(data, is_last_4=False))


        elif id == "403":
            data = [line[5:21][i:i+2] for i in range(0, len(line[5:21]), 2)]
            velocity = abs(ieee754_to_decimal(data, is_last_4=True)*3.6)
            vehicle_velocity.append(velocity)
            if velocity > 7:
                nr_vehicle_velocity_samples += 1
                vehicle_velocity_avg += velocity
            motor_velocity.append(abs(ieee754_to_decimal(data, is_last_4=False)))

        elif id == "40B":
            data = [line[5:21][i:i+2] for i in range(0, len(line[5:21]), 2)]
            heatsink_temp.append(ieee754_to_decimal(data, is_last_4=True))
            motor_temp.append(ieee754_to_decimal(data, is_last_4=False))

        elif id == "40E":
            data = [line[5:21][i:i+2] for i in range(0, len(line[5:21]), 2)]
            distance_odometer = ieee754_to_decimal(data, is_last_4=False)
            odometer.append(distance_odometer)
            if len(odometer) == 0:
                odometer.append(0)
            else:
                odometer.append(distance_odometer - odometer[-1])


# SAVING DATA TO CSV FILE
min_length = min(len(bus_current), len(pack_current), len(pack_open_voltage), len(vehicle_velocity), len(odometer))

data_dict = {
    'Bus Current': bus_current[:min_length],
    'Pack Current': pack_current[:min_length],
    'Pack Voltage': pack_open_voltage[:min_length],
    'Vehicle Velocity': vehicle_velocity[:min_length],
    'Distance Driven': odometer[:min_length]
}
df = pd.DataFrame(data_dict)
df.to_csv('vehicle_data_gunpoint_sträcka_Max1.csv', index=False)
