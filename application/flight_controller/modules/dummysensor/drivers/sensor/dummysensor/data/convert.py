import csv
import os

# Get the directory where this script is located
base_dir = os.path.dirname(os.path.abspath(__file__))

# List only directories
folders = [f for f in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, f))]

print(folders)
for folder in folders:
    imu_raw = open(f"{base_dir}/{folder}/imu.csv", "r")
    baro_raw = open(f"{base_dir}/{folder}/baro.csv", "r")

    out = open(f"{folder}.h", "w")

    imu_reader = csv.reader(imu_raw)
    baro_reader = csv.reader(baro_raw)

    out.write("#pragma once\n")
    out.write("#include <stdint.h>\n")
    out.write("#define DATA_DELAY 5\n")
    out.write("float imu_data[][7] = {\n")

    imu_length = 0
    for row in imu_reader:
        if row[1] == "ts":
            continue
        if row[1] != "IMU0":
            continue
        line = "{"
        line += row[0] + ","
        line += row[2] + ","
        line += row[3] + ","
        line += row[4] + ","
        line += row[5] + ","
        line += row[6] + ","
        line += row[7] + ","
        line += "},\n"
        out.write(line)
        imu_length += 1
    out.write("};\n")
    out.write(f"uint32_t imu_length = {imu_length};\n\n")

    out.write("float baro_data[][2] = {\n")
    baro_length = 0
    for row in baro_reader:
        if row[0] == "ts":
            continue
        if row[1] != "BARO0":
            continue
        line = "{" + str(float(row[0])) + "," +  str(float(row[3]) / 1000) + "},\n"
        out.write(line)
        baro_length += 1
    out.write("};\n\n")
    out.write(f"uint32_t baro_length = {baro_length};\n\n")
