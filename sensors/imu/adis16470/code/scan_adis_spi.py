#!/usr/bin/env python3

import time
import spidev

PROD_ID_CMD = 0x7200
EXPECTED = 0x4056


def word_to_bytes(word):
    return [(word >> 8) & 0xFF, word & 0xFF]


def bytes_to_word(rx):
    return (rx[0] << 8) | rx[1]


def xfer16(spi, word):
    rx = spi.xfer2(word_to_bytes(word))
    return bytes_to_word(rx)


def read_normal(spi):
    xfer16(spi, PROD_ID_CMD)
    time.sleep(0.001)
    return xfer16(spi, 0x0000)


def read_32_clock_one_cs(spi):
    rx = spi.xfer2([0x72, 0x00, 0x00, 0x00])
    return (rx[2] << 8) | rx[3]


def read_pipeline_3_words(spi):
    rx = spi.xfer2([0x72, 0x00, 0x00, 0x00, 0x00, 0x00])
    value_1 = (rx[2] << 8) | rx[3]
    value_2 = (rx[4] << 8) | rx[5]
    return value_1, value_2


def open_spi(bus, device, mode, speed):
    spi = spidev.SpiDev()
    spi.open(bus, device)
    spi.mode = mode
    spi.max_speed_hz = speed
    spi.bits_per_word = 8
    spi.no_cs = False
    spi.lsbfirst = False
    time.sleep(0.2)
    return spi


def test_setting(bus, device, mode, speed):
    try:
        spi = open_spi(bus, device, mode, speed)
    except FileNotFoundError:
        return

    normal_vals = []
    one_cs_vals = []
    pipe_vals = []

    try:
        for _ in range(5):
            normal_vals.append(read_normal(spi))
            time.sleep(0.02)

        for _ in range(5):
            one_cs_vals.append(read_32_clock_one_cs(spi))
            time.sleep(0.02)

        for _ in range(5):
            pipe_vals.append(read_pipeline_3_words(spi))
            time.sleep(0.02)

    finally:
        spi.close()

    all_vals = normal_vals + one_cs_vals
    for pair in pipe_vals:
        all_vals.extend(pair)

    hit = EXPECTED in all_vals

    if hit:
        print()
        print("FOUND POSSIBLE GOOD SETTING")
        print(f"bus={bus} device={device} mode={mode} speed={speed}")
        print(f"normal: {[hex(v) for v in normal_vals]}")
        print(f"32clk : {[hex(v) for v in one_cs_vals]}")
        print(f"pipe  : {[(hex(a), hex(b)) for a, b in pipe_vals]}")
        print()

    else:
        print(
            f"bus={bus} dev={device} mode={mode} speed={speed} "
            f"normal={hex(normal_vals[-1])} 32clk={hex(one_cs_vals[-1])} "
            f"pipe={hex(pipe_vals[-1][0])},{hex(pipe_vals[-1][1])}"
        )


def main():
    buses = [0]
    devices = [0, 1]
    modes = [0, 1, 2, 3]
    speeds = [50000, 100000, 500000, 1000000]

    print("Scanning ADIS16470 SPI settings...")
    print("Looking for PROD_ID = 0x4056")
    print()

    for bus in buses:
        for device in devices:
            for mode in modes:
                for speed in speeds:
                    test_setting(bus, device, mode, speed)


if __name__ == "__main__":
    main()
