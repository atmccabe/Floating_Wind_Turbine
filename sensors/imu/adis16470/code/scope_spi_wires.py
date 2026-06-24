#!/usr/bin/env python3

import argparse
import time
import spidev


# ADIS16470 registers
DIAG_STAT = 0x02
X_GYRO_OUT = 0x06
Y_GYRO_OUT = 0x0A
Z_GYRO_OUT = 0x0E
X_ACCL_OUT = 0x12
Y_ACCL_OUT = 0x16
Z_ACCL_OUT = 0x1A
PROD_ID = 0x72

EXPECTED_PROD_ID = 0x4056


REGISTERS = [
    ("DIAG_STAT", DIAG_STAT),
    ("X_GYRO", X_GYRO_OUT),
    ("Y_GYRO", Y_GYRO_OUT),
    ("Z_GYRO", Z_GYRO_OUT),
    ("X_ACCL", X_ACCL_OUT),
    ("Y_ACCL", Y_ACCL_OUT),
    ("Z_ACCL", Z_ACCL_OUT),
    ("PROD_ID", PROD_ID),
]


def xfer16(spi, word):
    tx = [(word >> 8) & 0xFF, word & 0xFF]
    rx = spi.xfer2(tx)
    return (rx[0] << 8) | rx[1]


def read_reg16(spi, addr):
    # Safe read command. This does not write to the ADIS.
    xfer16(spi, (addr & 0x7F) << 8)
    time.sleep(0.00002)
    return xfer16(spi, 0x0000)


def main():
    parser = argparse.ArgumentParser(description="Generate SPI activity for probing ADIS16470 wires.")
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--speed", type=int, default=10000)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()

    spi = spidev.SpiDev()

    try:
        spi.open(args.bus, args.device)

        # ADIS16470 SPI mode 3
        spi.mode = 0b11
        spi.max_speed_hz = args.speed
        spi.bits_per_word = 8
        spi.lsbfirst = False
        spi.cshigh = False

        print("SPI wire activity test running.")
        print()
        print(f"Bus/device: /dev/spidev{args.bus}.{args.device}")
        print(f"Speed: {args.speed} Hz")
        print(f"Duration: infinite")
        print()
        print("Probe these:")
        print("  ADIS pin 2  SCLK  -> should pulse")
        print("  ADIS pin 3  CS    -> should drop low during transfers")
        print("  ADIS pin 6  DIN   -> MOSI data from Pi")
        print("  ADIS pin 4  DOUT  -> MISO data back from ADIS")
        print()
        print("Scope ground clip must go to ADIS GND / Pi GND.")
        print()

        start = time.monotonic()
        count = 0

        while True:
            values = {}

            for name, addr in REGISTERS:
                values[name] = read_reg16(spi, addr)

            if count % 10 == 0:
                prod = values["PROD_ID"]
                diag = values["DIAG_STAT"]

                print(
                    f"t={time.monotonic() - start:6.2f}s | "
                    f"DIAG_STAT=0x{diag:04X} | "
                    f"PROD_ID=0x{prod:04X}"
                )

                if prod == EXPECTED_PROD_ID:
                    print("SUCCESS: ADIS product ID detected.")

            count += 1
            time.sleep(0.05)

    finally:
        spi.close()
        print()
        print("Done.")


if __name__ == "__main__":
    main()
