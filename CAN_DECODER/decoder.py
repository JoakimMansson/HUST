import matplotlib.pyplot as plt

# Lists to store data
internal_temp = []
high_temp = []
low_temp = []
avg_temp = []

pack_current = []
pack_open_voltage = []
pack_SOC = []
avg_current = []
low_cell_voltage = []
high_cell_voltage = []
avg_cell_voltage = []

'''
internal_temp = np.append(internal_temp, line[7:9])
            high_temp = np.append(high_temp, line[9:11])
            low_temp = np.append(low_temp, line[11:13])
            avg_temp = np.append(avg_temp, line[13:15])
            failsafe_status = np.append(failsafe_status, line[15:17])
            high_thermistor_id = np.append(high_thermistor_id, line[17:19])
            low_thermistor_id = np.append(low_thermistor_id, line[19:21])
'''

with open("2023-10-13-16-21-14.txt", "r") as f:
    for line in f:
        id = line[1:4]

        if id == "601" or id == "602":

            if id == "601":
                # Process CAN ID 601 data
                internal_temp.append(int(line[7:9], 16))
                high_temp.append(int(line[9:11], 16))
                low_temp.append(int(line[11:13], 16))
                avg_temp.append(int(line[13:15], 16))

            elif id == "602":
                # Process CAN ID 602 data
                pack_current.append(int(line[7:9], 16)*0.1)
                pack_open_voltage.append(int(line[9:11], 16))
                pack_SOC.append(int(line[11:13], 16)*0.5)
                avg_current.append(int(line[13:15], 16)*0.1)
                low_cell_voltage.append(int(line[15:17], 16)*0.0001)
                high_cell_voltage.append(int(line[17:19], 16)*0.0001)
                avg_cell_voltage.append(int(line[19:21], 16)*0.0001)


# Plot the data for CAN ID 601
plt.figure()
plt.plot(internal_temp, label="Internal Temperature")
plt.plot(high_temp, label="High Temperature")
plt.plot(low_temp, label="Low Temperature")
plt.plot(avg_temp, label="Average Temperature")
plt.xlabel("Sample")
plt.ylabel("Temperature C")
plt.title("Temperatures")
plt.legend()

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
plt.plot(pack_SOC, label="Pack SOC")
plt.xlabel("Sample")
plt.ylabel("Voltage")
plt.title("Pack Open Voltage")
plt.legend()

# Plot the data for pack current & avg_current
plt.figure()
plt.plot(pack_current, label="Pack Current")
plt.plot(avg_current, label="avg_current")
plt.xlabel("Sample")
plt.ylabel("Ampere")
plt.title("Pack Current")
plt.legend()

# Plot the data for pack SOC
plt.figure()
plt.plot(pack_SOC, label="Pack SOC")
plt.xlabel("Sample")
plt.ylabel("SOC")
plt.title("State Of Charge")
plt.legend()


plt.show()