#!/usr/bin/env python3

import time
import math
import spidev

DIAG_STAT = 0x02

X_GYRO_OUT = 0x06
Y_GYRO_OUT = 0x0A
Z_GYRO_OUT = 0x0E

X_ACCL_OUT = 0x12
Y_ACCL_OUT = 0x16
Z_ACCL_OUT = 0x1A

TEMP_OUT = 0x1C
PROD_ID = 0x72

EXPECTED_PROD_ID = 0x4056
G = 9.80665


def signed16(value):
    value = value & 0xFFFF
    if value & 0x8000:
        return value - 0x10000
    return value


def xfer16(spi, word):
    tx = [(word >> 8) & 0xFF, word & 0xFF]
    rx = spi.xfer2(tx)
    return (rx[0] << 8) | rx[1]


def read_reg16(spi, addr):
    # ADIS reads are pipelined:
    # 1st transfer asks for register
    # 2nd transfer gets the answer
    xfer16(spi, (addr & 0x7F) << 8)
    time.sleep(0.00005)
    return xfer16(spi, 0x0000)


spi = spidev.SpiDev()

try:
    spi.open(0, 0)
    spi.mode = 0b11
    spi.max_speed_hz = 100000
    spi.bits_per_word = 8
    spi.lsbfirst = False
    spi.cshigh = False

    print("ADIS16470 live test")
    print("Press Ctrl+C to stop.")
    print()

    # Clear SPI pipeline
    xfer16(spi, 0x0000)
    time.sleep(0.01)
    xfer16(spi, 0x0000)
    time.sleep(0.01)

    prod = read_reg16(spi, PROD_ID)
    print(f"PROD_ID = 0x{prod:04X}  expected 0x{EXPECTED_PROD_ID:04X}")
    print()

    print("sample | PROD | DIAG | gx dps | gy dps | gz dps | ax m/s2 | ay m/s2 | az m/s2")
    print("-" * 90)

    sample = 0

    while True:
        prod = read_reg16(spi, PROD_ID)
        diag = read_reg16(spi, DIAG_STAT)

        gx_raw = signed16(read_reg16(spi, X_GYRO_OUT))
        gy_raw = signed16(read_reg16(spi, Y_GYRO_OUT))
        gz_raw = signed16(read_reg16(spi, Z_GYRO_OUT))

        ax_raw = signed16(read_reg16(spi, X_ACCL_OUT))
        ay_raw = signed16(read_reg16(spi, Y_ACCL_OUT))
        az_raw = signed16(read_reg16(spi, Z_ACCL_OUT))

        gx_dps = gx_raw * 0.1
        gy_dps = gy_raw * 0.1
        gz_dps = gz_raw * 0.1

        ax_mps2 = ax_raw * 0.00125 * G
        ay_mps2 = ay_raw * 0.00125 * G
        az_mps2 = az_raw * 0.00125 * G

        print(
            f"{sample:6d} | "
            f"0x{prod:04X} | "
            f"0x{diag:04X} | "
            f"{gx_dps:7.2f} | "
            f"{gy_dps:7.2f} | "
            f"{gz_dps:7.2f} | "
            f"{ax_mps2:7.2f} | "
            f"{ay_mps2:7.2f} | "
            f"{az_mps2:7.2f}"
        )

        sample += 1
        time.sleep(0.25)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    spi.close()
