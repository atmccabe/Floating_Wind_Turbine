#!/usr/bin/env python3

import time
import spidev

PROD_ID = 0x72

spi = spidev.SpiDev()
spi.open(0, 0)

spi.mode = 0b11
spi.max_speed_hz = 10000
spi.bits_per_word = 8
spi.lsbfirst = False

def read_reg(addr):
    spi.xfer2([addr & 0x7F, 0x00])
    time.sleep(0.0001)

    data = spi.xfer2([0x00, 0x00])
    time.sleep(0.0001)

    return ((data[0] << 8) | data[1]) & 0xFFFF

try:
    while True:
        value = read_reg(PROD_ID)
        print(f"ADIS PROD_ID = 0x{value:04X}")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    spi.close()

