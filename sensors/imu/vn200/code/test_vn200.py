#!/usr/bin/env python3

import argparse
import glob
import time

import serial
from serial.tools import list_ports


BAUD_RATES = [
    115200,
    921600,
    460800,
    230400,
    57600,
    38400,
    19200,
    9600,
]


def calculate_checksum(message):
    checksum = 0

    for character in message:
        checksum ^= ord(character)

    return f"{checksum:02X}"


def make_command(message):
    checksum = calculate_checksum(message)
    return f"${message}*{checksum}\r\n".encode("ascii")


def find_serial_ports():
    ports = []

    for port in list_ports.comports():
        ports.append(port.device)

    patterns = [
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/serial*",
        "/dev/cu.usb*",
    ]

    for pattern in patterns:
        ports.extend(glob.glob(pattern))

    return list(dict.fromkeys(ports))


def parse_register_number(line):
    try:
        packet = line.split("*", 1)[0]
        fields = packet.lstrip("$").split(",")

        if len(fields) < 2:
            return None

        if fields[0] != "VNRRG":
            return None

        return int(fields[1])

    except (ValueError, IndexError):
        return None


def query_register(serial_port, register, timeout=1.5):
    command = make_command(f"VNRRG,{register:02d}")

    serial_port.write(command)
    serial_port.flush()

    end_time = time.monotonic() + timeout

    while time.monotonic() < end_time:
        raw_line = serial_port.readline()

        if not raw_line:
            continue

        line = raw_line.decode("ascii", errors="replace").strip()

        if not line:
            continue

        if line.startswith("$VNERR"):
            return line

        if parse_register_number(line) == register:
            return line

    return None


def test_connection(port, baud):
    try:
        with serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        ) as sensor:
            time.sleep(0.3)

            sensor.reset_input_buffer()
            sensor.reset_output_buffer()

            response = query_register(sensor, 1)

            if response is None:
                return None

            if response.startswith("$VNERR"):
                return None

            return response

    except (serial.SerialException, OSError):
        return None


def read_sensor_information(port, baud):
    register_names = {
        1: "Model number",
        2: "Hardware revision",
        3: "Serial number",
        4: "Firmware version",
    }

    with serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
    ) as sensor:
        time.sleep(0.3)
        sensor.reset_input_buffer()

        print("\nVN-200 information")
        print("------------------")

        for register, name in register_names.items():
            response = query_register(sensor, register)

            if response:
                print(f"{name}: {response}")
            else:
                print(f"{name}: no response")


def main():
    parser = argparse.ArgumentParser(
        description="Test a VectorNav VN-200 serial connection."
    )

    parser.add_argument(
        "--port",
        help="Serial port, such as /dev/ttyUSB0",
    )

    parser.add_argument(
        "--baud",
        type=int,
        help="Serial baud rate, such as 115200",
    )

    args = parser.parse_args()

    ports = [args.port] if args.port else find_serial_ports()
    baud_rates = [args.baud] if args.baud else BAUD_RATES

    if not ports:
        print("No serial ports were found.")
        print("Check the USB cable and run:")
        print("python3 -m serial.tools.list_ports -v")
        return

    print("Searching for VN-200...")
    print(f"Ports: {ports}")

    for port in ports:
        for baud in baud_rates:
            print(f"Trying {port} at {baud} baud...")

            response = test_connection(port, baud)

            if response:
                print("\nVN-200 connection found")
                print("-----------------------")
                print(f"Port: {port}")
                print(f"Baud: {baud}")
                print(f"Response: {response}")

                read_sensor_information(port, baud)
                return

    print("\nNo VN-200 response was found.")
    print("Possible causes:")
    print("1. Wrong serial interface or cable")
    print("2. Sensor is not powered")
    print("3. Incorrect serial port")
    print("4. Baud rate is not in the search list")
    print("5. Permission to access the port was denied")


if __name__ == "__main__":
    main()
