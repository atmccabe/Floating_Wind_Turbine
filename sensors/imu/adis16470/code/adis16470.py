import math
import time
import dataclasses import dataclass
import spidev

G = 9.80665

# ADIS16470 register addresses, low byte address of each 16-bit register
DIAG_STAT = 0x02

X_GYRO_OUT = 0x06
Y_GYRO_OUT = 0x0A
Z_GYRO_OUT = 0x0E

X_ACCL_OUT = 0x12
Y_ACCL_OUT = 0x16
Z_ACCL_OUT = 0x1A

TEMP_OUT = 0x1C
PROD_ID = 0x72

EXPECTED_PROD_ID = 0x4056  # decimal 16470

@dataclass
class ADISSample:
    diag_stat: int

    accel_x_mps2: float
    accel_y_mps2: float
    accel_z_mps2: float

    linear_accel_x_mps2: float
    linear_accel_y_mps2: float
    linear_accel_z_mps2: float

    gyro_x_radps: float
    gyro_y_radps: float
    gyro_z_radps: float

    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float

    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    temp_c: float

class ADIS16470:
    def __init__(
        self,
        bus: int = 0,
        device: int = 0,
        speed_hz: int = 1_000_000,
        complementary_alpha: float = 0.98,
    ):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)

        # ADIS16470 uses SPI mode 3: CPOL=1, CPHA=1
        self.spi.mode = 0b11

        # Datasheet says max 2 MHz; burst reads need <= 1 MHz.
        self.spi.max_speed_hz = speed_hz
        self.spi.lsbfirst = False

        self.alpha = complementary_alpha

        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self._last_time = None

    def close(self):
        self.spi.close()

    @staticmethod
    def _to_signed_16(value: int) -> int:
        value &= 0xFFFF
        if value & 0x8000:
            return value - 0x10000
        return value

    def _xfer16(self, word: int) -> int:
        tx = [(word >> 8) & 0xFF, word & 0xFF]
        rx = self.spi.xfer2(tx)
        return (rx[0] << 8) | rx[1]

    def read_reg16(self, addr: int) -> int:
        """
        Single 16-bit register read.

        ADIS register reads are pipelined:
        cycle 1 asks for the register,
        cycle 2 receives the result.
        """
        self._xfer16((addr & 0x7F) << 8)
        time.sleep(0.00002)
        return self._xfer16(0x0000)

    def read_regs16(self, addrs):
        """
        Read several 16-bit registers using pipelined SPI reads.
        """
        responses = []

        for addr in list(addrs) + [0x00]:
            responses.append(self._xfer16((addr & 0x7F) << 8))
            time.sleep(0.00002)

        # First response is from whatever was requested before.
        return responses[1:]

    def product_id(self) -> int:
        # Read twice just to clear any stale pipelined response.
        _ = self.read_reg16(PROD_ID)
        return self.read_reg16(PROD_ID)

    def check_connection(self) -> bool:
        return self.product_id() == EXPECTED_PROD_ID

    def read_sample(self) -> ADISSample:
        now = time.monotonic()

        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time

        self._last_time = now

        raw = self.read_regs16(
            [
                DIAG_STAT,
                X_GYRO_OUT,
                Y_GYRO_OUT,
                Z_GYRO_OUT,
                X_ACCL_OUT,
                Y_ACCL_OUT,
                Z_ACCL_OUT,
                TEMP_OUT,
            ]
        )

        diag = raw[0]

        gx_raw = self._to_signed_16(raw[1])
        gy_raw = self._to_signed_16(raw[2])
        gz_raw = self._to_signed_16(raw[3])

        ax_raw = self._to_signed_16(raw[4])
        ay_raw = self._to_signed_16(raw[5])
        az_raw = self._to_signed_16(raw[6])

        temp_raw = self._to_signed_16(raw[7])

        # ADIS16470 high-word scales:
        # gyro: 1 LSB = 0.1 deg/s
        # accel: 1 LSB = 1.25 mg
        gx_dps = gx_raw * 0.1
        gy_dps = gy_raw * 0.1
        gz_dps = gz_raw * 0.1

        gx_radps = math.radians(gx_dps)
        gy_radps = math.radians(gy_dps)
        gz_radps = math.radians(gz_dps)

        ax = ax_raw * 0.00125 * G
        ay = ay_raw * 0.00125 * G
        az = az_raw * 0.00125 * G

        temp_c = temp_raw * 0.1

        # Accel-based tilt estimate.
        # Assumes +Z sees about +1g when flat.
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if dt <= 0.0 or dt > 1.0:
            self.roll_rad = roll_acc
            self.pitch_rad = pitch_acc
        else:
            # Simple complementary filter:
            # gyro handles fast motion, accel slowly corrects drift.
            self.roll_rad = self.alpha * (self.roll_rad + gx_radps * dt) + (1.0 - self.alpha) * roll_acc
            self.pitch_rad = self.alpha * (self.pitch_rad + gy_radps * dt) + (1.0 - self.alpha) * pitch_acc
            self.yaw_rad += gz_radps * dt

        # Estimate gravity from roll/pitch, then subtract it.
        # This is not as good as the BNO085 quaternion gravity vector,
        # but it makes the ADIS CSV line up with the same plotting workflow.
        g_x = -G * math.sin(self.pitch_rad)
        g_y = G * math.sin(self.roll_rad) * math.cos(self.pitch_rad)
        g_z = G * math.cos(self.roll_rad) * math.cos(self.pitch_rad)

        lin_ax = ax - g_x
        lin_ay = ay - g_y
        lin_az = az - g_z

        return ADISSample(
            diag_stat=diag,

            accel_x_mps2=ax,
            accel_y_mps2=ay,
            accel_z_mps2=az,

            linear_accel_x_mps2=lin_ax,
            linear_accel_y_mps2=lin_ay,
            linear_accel_z_mps2=lin_az,

            gyro_x_radps=gx_radps,
            gyro_y_radps=gy_radps,
            gyro_z_radps=gz_radps,

            gyro_x_dps=gx_dps,
            gyro_y_dps=gy_dps,
            gyro_z_dps=gz_dps,

            roll_deg=math.degrees(self.roll_rad),
            pitch_deg=math.degrees(self.pitch_rad),
            yaw_deg=math.degrees(self.yaw_rad),

            temp_c=temp_c,
