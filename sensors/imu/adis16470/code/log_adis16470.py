#!/usr/bin/env python3

"""
ADIS16470 logger for Raspberry Pi.

This logs ADIS16470 IMU data to:

sensors/imu/adis16470/data/raw/adis16470_log_YYYYMMDD_HHMMSS.csv

The CSV is shaped to work like the other sensor logger/plotter code.

Important:
- ADIS16470 uses SPI, not I2C.
- ADIS16470 has gyro + accel.
- Roll and pitch are estimated.
- Yaw is gyro-integrated, so yaw will drift over time.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import spidev


# ----------------------------
# ADIS16470 register addresses
# ----------------------------

DIAG_STAT = 0x02

X_GYRO_OUT = 0x06
Y_GYRO_OUT = 0x0A
Z_GYRO_OUT = 0x0E

X_ACCL_OUT = 0x12
Y_ACCL_OUT = 0x16
Z_ACCL_OUT = 0x1A

TEMP_OUT = 0x1C
PROD_ID = 0x72

EXPECTED_PROD_ID = 0x4056  # 16470 decimal


# ----------------------------
# Unit conversion constants
# ----------------------------

G = 9.80665

# ADIS16470 high-word scale factors
GYRO_DPS_PER_LSB = 0.1          # deg/s per count
ACCEL_G_PER_LSB = 0.00125      # g per count
TEMP_C_PER_LSB = 0.1           # C per count, approximate


# ----------------------------
# File paths
# ----------------------------

REPO_ROOT = Path(__file__).resolve().parents[4]
RAW_DIR = REPO_ROOT / "sensors" / "imu" / "adis16470" / "data" / "raw"


FIELDNAMES = [
    "sample",
    "time_s",
    "sensor",

    "accel_x_mps2",
    "accel_y_mps2",
    "accel_z_mps2",

    "linear_accel_x_mps2",
    "linear_accel_y_mps2",
    "linear_accel_z_mps2",

    "gyro_x_radps",
    "gyro_y_radps",
    "gyro_z_radps",

    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",

    "roll_deg",
    "pitch_deg",
    "yaw_deg",

    "temp_c",
    "diag_stat",
]


# ----------------------------
# Helper functions
# ----------------------------

def to_signed_16(value):
    """Convert unsigned 16-bit value to signed 16-bit value."""
    value = value & 0xFFFF
    if value & 0x8000:
        return value - 0x10000
    return value


def make_output_path():
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return RAW_DIR / f"adis16470_log_{stamp}.csv"


class ADIS16470:
    def __init__(self, bus=0, device=0, speed_hz=1_000_000, alpha=0.98):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)

        # ADIS16470 uses SPI Mode 3
        self.spi.mode = 0b11

        # Keep this at or below 1 MHz for safer first testing
        self.spi.max_speed_hz = speed_hz
        self.spi.lsbfirst = False

        # Complementary filter value
        # Higher alpha = trust gyro more
        # Lower alpha = trust accel more
        self.alpha = alpha

        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self.last_time = None

    def close(self):
        self.spi.close()

    def xfer16(self, word):
        tx = [(word >> 8) & 0xFF, word & 0xFF]
        rx = self.spi.xfer2(tx)
        return (rx[0] << 8) | rx[1]

    def read_reg16(self, addr):
        """
        ADIS SPI reads are pipelined.

        First transfer asks for the register.
        Second transfer receives the answer.
        """
        self.xfer16((addr & 0x7F) << 8)
        time.sleep(0.00002)
        return self.xfer16(0x0000)

    def read_regs16(self, addrs):
        """
        Read multiple 16-bit registers.

        The first response is old/stale because ADIS is pipelined,
        so we throw away the first response.
        """
        responses = []

        for addr in list(addrs) + [0x00]:
            responses.append(self.xfer16((addr & 0x7F) << 8))
            time.sleep(0.00002)

        return responses[1:]

    def product_id(self):
        # Read twice to clear any old pipelined value
        self.read_reg16(PROD_ID)
        return self.read_reg16(PROD_ID)

    def read_sample(self):
        now = time.monotonic()

        if self.last_time is None:
            dt = 0.0
        else:
            dt = now - self.last_time

        self.last_time = now

        raw = self.read_regs16([
            DIAG_STAT,
            X_GYRO_OUT,
            Y_GYRO_OUT,
            Z_GYRO_OUT,
            X_ACCL_OUT,
            Y_ACCL_OUT,
            Z_ACCL_OUT,
            TEMP_OUT,
        ])

        diag_stat = raw[0]

        gx_raw = to_signed_16(raw[1])
        gy_raw = to_signed_16(raw[2])
        gz_raw = to_signed_16(raw[3])

        ax_raw = to_signed_16(raw[4])
        ay_raw = to_signed_16(raw[5])
        az_raw = to_signed_16(raw[6])

        temp_raw = to_signed_16(raw[7])

        # Convert gyro
        gyro_x_dps = gx_raw * GYRO_DPS_PER_LSB
        gyro_y_dps = gy_raw * GYRO_DPS_PER_LSB
        gyro_z_dps = gz_raw * GYRO_DPS_PER_LSB

        gyro_x_radps = math.radians(gyro_x_dps)
        gyro_y_radps = math.radians(gyro_y_dps)
        gyro_z_radps = math.radians(gyro_z_dps)

        # Convert accel
        accel_x_mps2 = ax_raw * ACCEL_G_PER_LSB * G
        accel_y_mps2 = ay_raw * ACCEL_G_PER_LSB * G
        accel_z_mps2 = az_raw * ACCEL_G_PER_LSB * G

        # Approx temp estimate
        temp_c = 25.0 + temp_raw * TEMP_C_PER_LSB

        # Accel-based roll and pitch estimate
        roll_acc = math.atan2(accel_y_mps2, accel_z_mps2)
        pitch_acc = math.atan2(
            -accel_x_mps2,
            math.sqrt(accel_y_mps2 ** 2 + accel_z_mps2 ** 2)
        )

        # First sample: initialize from accel
        if dt <= 0.0 or dt > 1.0:
            self.roll_rad = roll_acc
            self.pitch_rad = pitch_acc
        else:
            # Complementary filter:
            # gyro = fast motion
            # accel = slow gravity correction
            self.roll_rad = (
                self.alpha * (self.roll_rad + gyro_x_radps * dt)
                + (1.0 - self.alpha) * roll_acc
            )

            self.pitch_rad = (
                self.alpha * (self.pitch_rad + gyro_y_radps * dt)
                + (1.0 - self.alpha) * pitch_acc
            )

            # ADIS16470 does not have magnetometer correction.
            # This yaw value will drift over time.
            self.yaw_rad += gyro_z_radps * dt

        roll_deg = math.degrees(self.roll_rad)
        pitch_deg = math.degrees(self.pitch_rad)
        yaw_deg = math.degrees(self.yaw_rad)

        # Estimate gravity vector using roll/pitch
        gravity_x = -G * math.sin(self.pitch_rad)
        gravity_y = G * math.sin(self.roll_rad) * math.cos(self.pitch_rad)
        gravity_z = G * math.cos(self.roll_rad) * math.cos(self.pitch_rad)

        # Linear accel = measured accel - estimated gravity
        linear_accel_x_mps2 = accel_x_mps2 - gravity_x
        linear_accel_y_mps2 = accel_y_mps2 - gravity_y
        linear_accel_z_mps2 = accel_z_mps2 - gravity_z

        return {
            "accel_x_mps2": accel_x_mps2,
            "accel_y_mps2": accel_y_mps2,
            "accel_z_mps2": accel_z_mps2,

            "linear_accel_x_mps2": linear_accel_x_mps2,
            "linear_accel_y_mps2": linear_accel_y_mps2,
            "linear_accel_z_mps2": linear_accel_z_mps2,

            "gyro_x_radps": gyro_x_radps,
            "gyro_y_radps": gyro_y_radps,
            "gyro_z_radps": gyro_z_radps,

            "gyro_x_dps": gyro_x_dps,
            "gyro_y_dps": gyro_y_dps,
            "gyro_z_dps": gyro_z_dps,

            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "yaw_deg": yaw_deg,

            "temp_c": temp_c,
            "diag_stat": diag_stat,
        }


def main():
    parser = argparse.ArgumentParser(description="Log ADIS16470 IMU data to CSV.")

    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--speed", type=int, default=1_000_000)
    parser.add_argument("--alpha", type=float, default=0.98)
    parser.add_argument("--skip-id-check", action="store_true")

    args = parser.parse_args()

    if args.rate <= 0:
        print("ERROR: --rate must be greater than 0")
        sys.exit(1)

    output_path = make_output_path()

    imu = ADIS16470(
        bus=args.bus,
        device=args.device,
        speed_hz=args.speed,
        alpha=args.alpha,
    )

    try:
        if not args.skip_id_check:
            prod_id = imu.product_id()

            if prod_id != EXPECTED_PROD_ID:
                print("ERROR: ADIS16470 product ID check failed.")
                print(f"Expected: 0x{EXPECTED_PROD_ID:04X}")
                print(f"Got:      0x{prod_id:04X}")
                print()
                print("Check these things:")
                print("1. Pi is powered off when wiring.")
                print("2. ADIS VDD is connected to Pi 3.3V, not 5V.")
                print("3. ADIS GND is connected to Pi GND.")
                print("4. ADIS SCLK goes to Pi physical pin 23.")
                print("5. ADIS CS goes to Pi physical pin 24.")
                print("6. ADIS DOUT goes to Pi physical pin 21, MISO.")
                print("7. ADIS DIN goes to Pi physical pin 19, MOSI.")
                print("8. SPI is enabled with raspi-config.")
                print()
                print("For wiring debug only, you can try:")
                print("python3 sensors/imu/adis16470/code/log_adis16470.py --skip-id-check")
                sys.exit(1)

            print(f"ADIS16470 connected. PROD_ID = 0x{prod_id:04X}")

        print(f"Saving CSV to: {output_path}")
        print("Press Ctrl+C to stop.")
        print()

        period = 1.0 / args.rate
        start_time = time.monotonic()
        next_time = start_time
        sample_num = 0

        with output_path.open("w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=FIELDNAMES)
            writer.writeheader()

            while True:
                now = time.monotonic()
                time_s = now - start_time

                if args.duration is not None and time_s >= args.duration:
                    break

                data = imu.read_sample()

                row = {
                    "sample": sample_num,
                    "time_s": time_s,
                    "sensor": "adis16470",

                    "accel_x_mps2": data["accel_x_mps2"],
                    "accel_y_mps2": data["accel_y_mps2"],
                    "accel_z_mps2": data["accel_z_mps2"],

                    "linear_accel_x_mps2": data["linear_accel_x_mps2"],
                    "linear_accel_y_mps2": data["linear_accel_y_mps2"],
                    "linear_accel_z_mps2": data["linear_accel_z_mps2"],

                    "gyro_x_radps": data["gyro_x_radps"],
                    "gyro_y_radps": data["gyro_y_radps"],
                    "gyro_z_radps": data["gyro_z_radps"],

                    "gyro_x_dps": data["gyro_x_dps"],
                    "gyro_y_dps": data["gyro_y_dps"],
                    "gyro_z_dps": data["gyro_z_dps"],

                    "roll_deg": data["roll_deg"],
                    "pitch_deg": data["pitch_deg"],
                    "yaw_deg": data["yaw_deg"],

                    "temp_c": data["temp_c"],
                    "diag_stat": f"0x{data['diag_stat']:04X}",
                }

                writer.writerow(row)

                if sample_num % max(1, int(args.rate)) == 0:
                    print(
                        f"t={time_s:7.2f}s | "
                        f"pitch={data['pitch_deg']:8.2f} deg | "
                        f"roll={data['roll_deg']:8.2f} deg | "
                        f"yaw={data['yaw_deg']:8.2f} deg | "
                        f"az={data['accel_z_mps2']:8.2f} m/s^2"
                    )

                sample_num += 1

                next_time += period
                sleep_time = next_time - time.monotonic()

                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print()
        print("Stopped by user.")

    finally:
        imu.close()

    print()
    print("Done.")
    print(f"Saved file: {output_path}")


if __name__ == "__main__":
    main()
