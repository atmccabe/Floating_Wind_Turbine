#!/usr/bin/env python3

import time
import spidev


PROD_ID = 0x72
DIAG_STAT = 0x02

EXPECTED_PROD_ID = 0x4056


def xfer16(spi, word):
    tx = [(word >> 8) & 0xFF, word & 0xFF]
    rx = spi.xfer2(tx)
    return (rx[0] << 8) | rx[1]


def read_reg16(spi, addr):
    # ADIS reads are pipelined:
    # first transfer asks for the register
    # second transfer gets the result
    xfer16(spi, (addr & 0x7F) << 8)
    time.sleep(0.00002)
    return xfer16(spi, 0x0000)


spi = spidev.SpiDev()

try:
    spi.open(0, 0)
    spi.max_speed_hz = 100000
    spi.mode = 0b11
    spi.bits_per_word = 8
    spi.lsbfirst = False

    print("Testing ADIS16470 SPI connection...")
    print("Looking for PROD_ID = 0x4056")
    print()

    for i in range(10):
        diag = read_reg16(spi, DIAG_STAT)
        prod = read_reg16(spi, PROD_ID)

        print(f"read {i+1:02d}: DIAG_STAT=0x{diag:04X}  PROD_ID=0x{prod:04X}")

        if prod == EXPECTED_PROD_ID:
            print()
            print("SUCCESS: Raspberry Pi is reading the ADIS16470.")
            break

        time.sleep(0.2)

    else:
        print()
        print("FAILED: Did not read the expected ADIS16470 product ID.")
        print("Check power, ground, SCLK, CS, DOUT/MISO, and DIN/MOSI.")

finally:
    spi.close()
