import matplotlib.pyplot as plt
import struct
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


with open("2023-10-20/2023-10-13-16-21-14.txt", "r") as f:
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
            if is_start_odometer:
                start_distance = distance_odometer
                is_start_odometer = False


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
df.to_csv('vehicle_data.csv', index=False)
#model = joblib.load("linear_regression.pk1")


# Plot the data for CAN ID 601
plt.figure()
plt.plot(internal_temp, label="Internal Temperature BMS")
plt.plot(high_temp, label="Highest Temperature Battery")
plt.plot(low_temp, label="Lowest Temperature Battery")
plt.plot(avg_temp, label="Average Temperature Battery")
plt.xlabel("Sample")
plt.ylabel("Temperature C")
plt.title("Temperatures BMS & Battery")
plt.legend()

plt.figure()
plt.hist(high_thermistor_id, bins=20, alpha=0.5, label='High Thermistor IDs')
plt.hist(low_thermistor_id, bins=20, alpha=0.5, label='Low Thermistor IDs')
plt.legend(loc='upper right')
plt.xlabel("Thermistor IDs")
plt.ylabel("Frequency")
plt.title("High and Low Thermistor IDs Histogram")

plt.figure()
plt.hist(current_limit, bins=20, alpha=0.5, label='Current limits')
plt.xlabel("Current Limit")
plt.ylabel("Frequency")
plt.title("Current Limit Histogram")


# Plot the data for cell voltage
plt.figure()
plt.plot(low_cell_voltage, label="Low Cell Voltage")
plt.plot(high_cell_voltage, label="High Cell Voltage")
plt.plot(avg_cell_voltage, label="Avg Cell Voltage")
plt.xlabel("Sample")
plt.ylabel("Voltage")
plt.title("Cell Voltages")
plt.legend()

# Plot the data for pack open voltage
plt.figure()
plt.plot(pack_open_voltage, label="Pack Open Voltage")
plt.plot(pack_inst_voltage, label="Pack Inst Voltage")
plt.xlabel("Sample")
plt.ylabel("Voltage")
plt.title("Pack Voltages")
plt.legend()

# Plot the data for pack current & avg_current
plt.figure()
plt.plot(pack_current, label="Pack Current")
#plt.plot(avg_current, label="Average Current")
#plt.plot(pack_abs_current, label="Pack Abs Current")
plt.xlabel("Sample")
plt.ylabel("Ampere")
plt.title("Battery Pack Current")
plt.legend()

# Plot the data for pack SOC
plt.figure()
plt.plot(pack_SOC, label="Pack SOC")
plt.xlabel("Sample")
plt.ylabel("SOC (%)")
plt.title("State Of Charge")
plt.legend()


plt.figure()
plt.plot(vehicle_velocity, label="Vehicle Velocity")
plt.xlabel("Sample")
plt.ylabel("Velocity (km/h)")
plt.title("Velocity Vehicle")
plt.legend()

plt.figure()
plt.plot(motor_velocity, label="Motor Velocity")
plt.xlabel("Sample")
plt.ylabel("Motor Velocity (rpm)")
plt.title("Velocity RPM")
plt.legend()

plt.figure()
plt.plot(motor_temp, label="Temperature Motor")
plt.plot(heatsink_temp, label="Heatsink Temperature")
plt.xlabel("Sample")
plt.ylabel("Temperature (C)")
plt.title("Motorcontroller Temperatures")
plt.legend()


plt.figure()
plt.plot(bus_current, label="Bus Current")
plt.xlabel("Sample")
plt.ylabel("Ampere")
plt.title("Motorcontroller Bus Current")
plt.legend()

plt.figure()
plt.plot(bus_voltage, label="Bus Voltage")
plt.xlabel("Sample")
plt.ylabel("Voltage")
plt.title("Motorcontroller Bus Voltage")
plt.legend()

print(f"Distance Travelled: {(odometer[len(odometer)-1] - start_distance)/1000} km")
print(f"Average Bus Current: {bus_current_avg/nr_bus_current_samples} A")
print(f"Average Pack Current: {pack_current_avg/nr_pack_current_samples} A")
print(f"Average Vehicle Velocity: {vehicle_velocity_avg/nr_vehicle_velocity_samples} km/h")
plt.show()