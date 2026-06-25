import math
import time


class Sensor:
    name = "adis16470"

    csv_fields = [
        "accel_x_g",
        "accel_y_g",
        "accel_z_g",
        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",
        "gyro_x_dps",
        "gyro_y_dps",
        "gyro_z_dps",
        "gyro_x_radps",
        "gyro_y_radps",
        "gyro_z_radps",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "accel_roll_deg",
        "accel_pitch_deg",
        "temp_c",
        "diag_stat",
        "prod_id",
    ]

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

    GYRO_SCALE_DPS = 0.1
    ACCEL_SCALE_G = 0.00125
    TEMP_SCALE_C = 0.1

    G_TO_MPS2 = 9.80665
    DPS_TO_RADPS = math.pi / 180.0

    def __init__(self):
        self.spi = None

        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.zero_roll = 0.0
        self.zero_pitch = 0.0
        self.zero_yaw = 0.0

        self.last_time = None
        self.has_first_angle = False

        self.alpha = 0.98

    def connect(self):
        import spidev

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)

        self.spi.mode = 0b11
        self.spi.max_speed_hz = 10000
        self.spi.bits_per_word = 8
        self.spi.lsbfirst = False

        time.sleep(0.5)

        prod_id = self.check_product_id()

        if prod_id != self.EXPECTED_PROD_ID:
            raise RuntimeError(
                f"ADIS16470 product ID did not match. Got 0x{prod_id:04X}, expected 0x4056."
            )

        self.calibrate_gyro()
        self.last_time = time.monotonic()

    def check_product_id(self):
        prod_id = 0x0000

        for attempt in range(10):
            prod_id = self.read_u16(self.PROD_ID)
            print(f"ADIS16470 PROD_ID attempt {attempt + 1}: 0x{prod_id:04X}")

            if prod_id == self.EXPECTED_PROD_ID:
                return prod_id

            time.sleep(0.1)

        return prod_id

    def read_u16(self, address):
        self.spi.xfer2([address & 0x7F, 0x00])
        time.sleep(0.0001)

        data = self.spi.xfer2([0x00, 0x00])
        time.sleep(0.0001)

        return ((data[0] << 8) | data[1]) & 0xFFFF

    def read_s16(self, address):
        value = self.read_u16(address)

        if value & 0x8000:
            value -= 0x10000

        return value

    def calibrate_gyro(self):
        print("Hold ADIS16470 still for gyro zeroing...")

        samples = 100
        sx = 0.0
        sy = 0.0
        sz = 0.0

        for _ in range(samples):
            sx += self.read_s16(self.X_GYRO_OUT) * self.GYRO_SCALE_DPS
            sy += self.read_s16(self.Y_GYRO_OUT) * self.GYRO_SCALE_DPS
            sz += self.read_s16(self.Z_GYRO_OUT) * self.GYRO_SCALE_DPS
            time.sleep(0.01)

        self.gyro_bias_x = sx / samples
        self.gyro_bias_y = sy / samples
        self.gyro_bias_z = sz / samples

        print(
            "Gyro bias dps: "
            f"x={self.gyro_bias_x:.3f}, "
            f"y={self.gyro_bias_y:.3f}, "
            f"z={self.gyro_bias_z:.3f}"
        )

    def accel_angles(self, ax_g, ay_g, az_g):
        roll = math.degrees(math.atan2(ay_g, az_g))

        pitch = math.degrees(
            math.atan2(
                -ax_g,
                math.sqrt((ay_g * ay_g) + (az_g * az_g))
            )
        )

        return roll, pitch

    def zero(self):
        self.zero_roll = self.roll
        self.zero_pitch = self.pitch
        self.zero_yaw = self.yaw
        return True

    def read(self):
        now = time.monotonic()

        if self.last_time is None:
            dt = 0.02
        else:
            dt = now - self.last_time

        self.last_time = now

        if dt <= 0:
            dt = 0.02

        diag_stat = self.read_u16(self.DIAG_STAT)
        prod_id = self.read_u16(self.PROD_ID)

        gx_dps = self.read_s16(self.X_GYRO_OUT) * self.GYRO_SCALE_DPS
        gy_dps = self.read_s16(self.Y_GYRO_OUT) * self.GYRO_SCALE_DPS
        gz_dps = self.read_s16(self.Z_GYRO_OUT) * self.GYRO_SCALE_DPS

        ax_g = self.read_s16(self.X_ACCL_OUT) * self.ACCEL_SCALE_G
        ay_g = self.read_s16(self.Y_ACCL_OUT) * self.ACCEL_SCALE_G
        az_g = self.read_s16(self.Z_ACCL_OUT) * self.ACCEL_SCALE_G

        temp_c = self.read_s16(self.TEMP_OUT) * self.TEMP_SCALE_C

        gx_dps -= self.gyro_bias_x
        gy_dps -= self.gyro_bias_y
        gz_dps -= self.gyro_bias_z

        accel_roll, accel_pitch = self.accel_angles(ax_g, ay_g, az_g)

        if not self.has_first_angle:
            self.roll = accel_roll
            self.pitch = accel_pitch
            self.yaw = 0.0
            self.has_first_angle = True
        else:
            self.roll = self.alpha * (self.roll + gx_dps * dt) + (1.0 - self.alpha) * accel_roll
            self.pitch = self.alpha * (self.pitch + gy_dps * dt) + (1.0 - self.alpha) * accel_pitch
            self.yaw = self.yaw + gz_dps * dt

        roll_out = self.roll - self.zero_roll
        pitch_out = self.pitch - self.zero_pitch
        yaw_out = self.yaw - self.zero_yaw

        return {
            "accel_x_g": ax_g,
            "accel_y_g": ay_g,
            "accel_z_g": az_g,

            "accel_x_mps2": ax_g * self.G_TO_MPS2,
            "accel_y_mps2": ay_g * self.G_TO_MPS2,
            "accel_z_mps2": az_g * self.G_TO_MPS2,

            "gyro_x_dps": gx_dps,
            "gyro_y_dps": gy_dps,
            "gyro_z_dps": gz_dps,

            "gyro_x_radps": gx_dps * self.DPS_TO_RADPS,
            "gyro_y_radps": gy_dps * self.DPS_TO_RADPS,
            "gyro_z_radps": gz_dps * self.DPS_TO_RADPS,

            "roll_deg": roll_out,
            "pitch_deg": pitch_out,
            "yaw_deg": yaw_out,

            "accel_roll_deg": accel_roll,
            "accel_pitch_deg": accel_pitch,

            "temp_c": temp_c,
            "diag_stat": f"0x{diag_stat:04X}",
            "prod_id": f"0x{prod_id:04X}",
        }
