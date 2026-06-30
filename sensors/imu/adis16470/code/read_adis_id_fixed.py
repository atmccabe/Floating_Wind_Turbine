#!/usr/bin/env python3

import time
import argparse
import spidev

EXPECTED_PROD_ID = 0x4056


def open_spi(bus, device, speed):
    spi = spidev.SpiDev()
    spi.open(bus, device)

    spi.mode = 3
    spi.max_speed_hz = speed
    spi.bits_per_word = 8
    spi.lsbfirst = False
    spi.no_cs = False
    spi.cshigh = False

    time.sleep(1.0)
    return spi


def xfer2_bytes(spi, data):
    return spi.xfer2(list(data))


def read_prod_id(spi):
    # Frame 1: ask for PROD_ID at address 0x72
    rx_cmd = xfer2_bytes(spi, [0x72, 0x00])
    time.sleep(0.002)

    # Frame 2: dummy clocks, answer comes back here
    rx_ans = xfer2_bytes(spi, [0x00, 0x00])
    time.sleep(0.002)

    value = (rx_ans[0] << 8) | rx_ans[1]
    return value, rx_cmd, rx_ans


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--speed", type=int, default=100000)
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--delay", type=float, default=0.25)
    args = parser.parse_args()

    spi = open_spi(args.bus, args.device, args.speed)

    print("ADIS16470 simple PROD_ID test")
    print(f"SPI bus={args.bus}, device={args.device}, speed={args.speed} Hz, mode=3")
    print("Expected: 0x4056")
    print()

    good = 0

    try:
        for i in range(args.count):
            value, rx_cmd, rx_ans = read_prod_id(spi)

            if value == EXPECTED_PROD_ID:
                status = "OK"
                good += 1
            else:
                status = "BAD"

            print(
                f"{i:03d}: PROD_ID = 0x{value:04X}  {status}   "
                f"cmd_rx={rx_cmd[0]:02X} {rx_cmd[1]:02X}   "
                f"ans_rx={rx_ans[0]:02X} {rx_ans[1]:02X}"
            )

            time.sleep(args.delay)

    finally:
        spi.close()

    print()
    print(f"Good reads: {good}/{args.count}")


if __name__ == "__main__":
    main()
