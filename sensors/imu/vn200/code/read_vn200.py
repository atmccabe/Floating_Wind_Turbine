#!/usr/bin/env python3

import argparse
import time

import serial


def calculate_checksum(message):
    checksum = 0

    for character in message:
        checksum ^= ord(character)

    return f"{checksum:02X}"


def make_command(message):
    checksum = calculate_checksum(message)
    return f"${message}*{checksum}\r\n".encode("ascii")


def remove_checksum(line):
    return line.split("*", 1)[0].lstrip("$")


def parse_motion_packet(line):
    body = remove_checksum(line)
    fields = body.split(",")

    # Response from reading register 27
    if len(fields) >= 14 and fields[0] == "VNRRG" and fields[1] == "27":
        values = fields[2:14]

    # Asynchronous VNYMR packet
    elif len(fields) >= 13 and fields[0] == "VNYMR":
        values = fields[1:13]

    else:
        return None

    try:
        numbers = [float(value) for value in values]

    except ValueError:
        return None

    return {
        "yaw_deg": numbers[0],
        "pitch_deg": numbers[1],
        "roll_deg": numbers[2],
        "mag_x_gauss": numbers[3],
        "mag_y_gauss": numbers[4],
        "mag_z_gauss": numbers[5],
        "accel_x_mps2": numbers[6],
        "accel_y_mps2": numbers[7],
        "accel_z_mps2": numbers[8],
        "gyro_x_radps": numbers[9],
        "gyro_y_radps": numbers[10],
        "gyro_z_radps": numbers[11],
    }


def query_motion_data(sensor, timeout=0.25):
    command = make_command("VNRRG,27")

    sensor.write(command)
    sensor.flush()

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        raw_line = sensor.readline()

        if not raw_line:
            continue

        line = raw_line.decode("ascii", errors="replace").strip()

        if not line:
            continue

        if line.startswith("$VNERR"):
            print(f"Sensor error: {line}")
            return None

        data = parse_motion_packet(line)

        if data is not None:
            return data

    return None


def main():
    parser = argparse.ArgumentParser(
        description="Read live orientation and IMU data from a VN-200."
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Requested reading rate in Hz",
    )

    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("Rate must be greater than zero.")

    period = 1.0 / args.rate

    print("Connecting to VN-200...")
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Rate: {args.rate:.1f} Hz")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
    ) as sensor:
        time.sleep(0.5)

        sensor.reset_input_buffer()
        sensor.reset_output_buffer()

        start_time = time.monotonic()
        next_sample_time = start_time
        sample = 0

        try:
            while True:
                now = time.monotonic()

                if now < next_sample_time:
                    time.sleep(next_sample_time - now)

                data = query_motion_data(sensor)

                if data is None:
                    print("No motion-data response.")
                else:
                    elapsed = time.monotonic() - start_time

                    print(
                        f"{sample:06d} | "
                        f"t={elapsed:8.3f} s | "
                        f"yaw={data['yaw_deg']:8.3f}° | "
                        f"pitch={data['pitch_deg']:8.3f}° | "
                        f"roll={data['roll_deg']:8.3f}° | "
                        f"accel=({data['accel_x_mps2']:7.3f}, "
                        f"{data['accel_y_mps2']:7.3f}, "
                        f"{data['accel_z_mps2']:7.3f}) m/s² | "
                        f"gyro=({data['gyro_x_radps']:7.3f}, "
                        f"{data['gyro_y_radps']:7.3f}, "
                        f"{data['gyro_z_radps']:7.3f}) rad/s"
                    )

                    sample += 1

                next_sample_time += period

                # Recover if the program falls behind
                if time.monotonic() - next_sample_time > period:
                    next_sample_time = time.monotonic()

        except KeyboardInterrupt:
            print("\nVN-200 reading stopped.")
            print(f"Samples read: {sample}")


if __name__ == "__main__":
    main()
