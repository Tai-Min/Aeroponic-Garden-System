import serial
import csv
from os.path import exists

label = "mould"
num_files = 1500
csv_header = ["timestamp", "temperature", "pressure", "humidity", "gas"]
serial_port = "COM13"
serial_baud = 115200

def reading_to_lists(reading):
    try:
        res = reading.decode("utf-8").strip()
        res = [float(val) for val in res.split(",")]
    except:
        return None

    if len(res) != (len(csv_header) - 1):
        return None

    return res


def find_available_filename():
    fname = "./dataset/" + label + "." + str(find_available_filename.cntr) + ".csv"

    while exists(fname):
        find_available_filename.cntr += 1
        fname = "./dataset/" + label + "." + str(find_available_filename.cntr) + ".csv"

    return fname
find_available_filename.cntr = 1

ser = serial.Serial()
ser.baudrate = serial_baud
ser.port = serial_port
ser.open()

curr_files = 0

while curr_files < num_files:
    sampling_success = True

    ser.flush()

    reading = ser.readline()
    reading_parsed = reading_to_lists(reading)

    if not reading_parsed:
        continue

    reading_parsed.insert(0, 0)

    print(reading_parsed)

    with open(find_available_filename(), "a", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_header)
        writer.writerow(reading_parsed)
        curr_files += 1