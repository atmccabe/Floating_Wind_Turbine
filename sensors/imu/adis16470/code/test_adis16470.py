import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path
import spidev

# ADSI16470 spec

PROD_ID_ADDR = 0x72
EXPECTED_PROD_ID = 0x4056
BURST_READ_CMD = 0x6800
GYRO_SCALE_DPS = 0.1 #1 count = 0.1 deg/s
ACCEL_SCALE_G = 0.00125 #1 count = 0.00125 g
G_TO_MPS2 = 9.80665

SENSOR_ROOT = Path(__file__).resolve().parents[1]
RAW_DATA_DIR = SENSOR_ROOT / "data" / "raw"

# Converts 16 bit unsigned into signed data
def to_signed_16 (value):
	if value & 0x8000:
		return value - 0x10000
	return value

# Sends 16 bit SPI words and get 16 bit words back
def spi_transfer_words(spi,words):
	tx_bytes = []
	for word in words:
		tx_bytes.append((word>>8)& 0xFF)
		tx_bytes.append(word & 0xFF)

	rx_bytes = spi.xfer2(tx_bytes)
	rx_words = []

	for i in range (0,len(rx_bytes), 2):
		word = (rx_bytes[i] << 8) | rx_bytes[i+1]
		rx_words.append(word)
	return rx_words

# Reads ADIS register
def read_register(spi, address):
	command = (address & 0x7F) << 8
	spi_transfer_words(spi, [command])
	value = spi_transfer_words(spi, [0x0000])[0]
	return value

# Checks if the sensors is ADIS sensor
def check_product_id(spi):
	product_id = read_register(spi, PROD_ID_ADDR)
	print(f"Product ID : 0x{product_id:04X}")
	if product_id != EXPECTED_PROD_ID:
		print("ADIS16460 not detected")
		print("Expected Product ID = 0x4056")
		print("Check power, gnd, wiring,and SPI enable")
		return False
	return True

# Calculates burst checksum
def calculate_checksum(words):
    total = 0

    for word in words:
        total += (word >> 8) & 0xFF
        total += word & 0xFF

    return total & 0xFFFF


# Reads one full accel/gyro burst packet
def read_burst(spi):
    rx = spi_transfer_words(spi, [BURST_READ_CMD] + [0x0000] * 10)

    data = rx[1:]

    diag_stat = data[0]

    gyro_x_raw = to_signed_16(data[1])
    gyro_y_raw = to_signed_16(data[2])
    gyro_z_raw = to_signed_16(data[3])

    accel_x_raw = to_signed_16(data[4])
    accel_y_raw = to_signed_16(data[5])
    accel_z_raw = to_signed_16(data[6])

    data_counter = data[8]
    checksum_sensor = data[9]
    checksum_calc = calculate_checksum(data[:9])
    checksum_ok = checksum_sensor == checksum_calc

    return {
        "diag_stat": diag_stat,
        "gyro_x_raw": gyro_x_raw,
        "gyro_y_raw": gyro_y_raw,
        "gyro_z_raw": gyro_z_raw,
        "accel_x_raw": accel_x_raw,
        "accel_y_raw": accel_y_raw,
        "accel_z_raw": accel_z_raw,
        "data_counter": data_counter,
        "checksum_ok": checksum_ok,
    }


# Estimates roll and pitch from gravity
def accel_to_roll_pitch(ax, ay, az):
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

    return roll, pitch


# Makes a new CSV file path
def make_csv_file():
    RAW_DATA_DIR.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"adis16470_log_{timestamp}.csv"

    return RAW_DATA_DIR / filename


# Main program
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()

    spi = spidev.SpiDev()
    spi.open(0, 0)

    spi.mode = 0b11
    spi.max_speed_hz = 1_000_000
    spi.bits_per_word = 8

    time.sleep(0.25)

    if not check_product_id(spi):
        spi.close()
        return

    csv_file = make_csv_file()

    print(f"Saving to: {csv_file}")
    print("Press Ctrl+C to stop.")
    print()

    fieldnames = [
        "sample",
        "time_s",

        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",

        "gyro_x_radps",
        "gyro_y_radps",
        "gyro_z_radps",

        "roll_deg",
        "pitch_deg",
        "yaw_deg",

        "data_counter",
        "diag_stat",
        "checksum_ok",
    ]

    start_time = time.monotonic()
    last_time = start_time
    sample = 0
    yaw_deg = 0.0

    sample_period = 1.0 / args.rate

    try:
        with open(csv_file, "w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()

            while True:
                now = time.monotonic()
                time_s = now - start_time
                dt = now - last_time
                last_time = now

                data = read_burst(spi)

                accel_x_g = data["accel_x_raw"] * ACCEL_SCALE_G
                accel_y_g = data["accel_y_raw"] * ACCEL_SCALE_G
                accel_z_g = data["accel_z_raw"] * ACCEL_SCALE_G

                accel_x = accel_x_g * G_TO_MPS2
                accel_y = accel_y_g * G_TO_MPS2
                accel_z = accel_z_g * G_TO_MPS2

                gyro_x_dps = data["gyro_x_raw"] * GYRO_SCALE_DPS
                gyro_y_dps = data["gyro_y_raw"] * GYRO_SCALE_DPS
                gyro_z_dps = data["gyro_z_raw"] * GYRO_SCALE_DPS

                gyro_x = math.radians(gyro_x_dps)
                gyro_y = math.radians(gyro_y_dps)
                gyro_z = math.radians(gyro_z_dps)

                roll_deg, pitch_deg = accel_to_roll_pitch(accel_x, accel_y, accel_z)

                yaw_deg += gyro_z_dps * dt

                writer.writerow({
                    "sample": sample,
                    "time_s": f"{time_s:.6f}",

                    "accel_x_mps2": f"{accel_x:.6f}",
                    "accel_y_mps2": f"{accel_y:.6f}",
                    "accel_z_mps2": f"{accel_z:.6f}",

                    "gyro_x_radps": f"{gyro_x:.6f}",
                    "gyro_y_radps": f"{gyro_y:.6f}",
                    "gyro_z_radps": f"{gyro_z:.6f}",

                    "roll_deg": f"{roll_deg:.3f}",
                    "pitch_deg": f"{pitch_deg:.3f}",
                    "yaw_deg": f"{yaw_deg:.3f}",

                    "data_counter": data["data_counter"],
                    "diag_stat": f"0x{data['diag_stat']:04X}",
                    "checksum_ok": data["checksum_ok"],
                })

                if sample % 10 == 0:
                    print(
                        f"t={time_s:6.2f}s | "
                        f"roll={roll_deg:7.2f} | "
                        f"pitch={pitch_deg:7.2f} | "
                        f"yaw={yaw_deg:7.2f} | "
                        f"checksum={data['checksum_ok']}"
                    )

                sample += 1

                if args.duration is not None and time_s >= args.duration:
                    break

                time.sleep(sample_period)

    except KeyboardInterrupt:
        print()
        print("Stopped.")

    spi.close()
    print(f"Saved: {csv_file}")


main()
