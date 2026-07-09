import glob
import os
import time

import serial


class Sensor:
    def __init__(self):
        self.name = "vn200"
        self.serial_port = None

        self.csv_fields = [
            "message_type",

            "yaw_deg",
            "pitch_deg",
            "roll_deg",

            "mag_x_gauss",
            "mag_y_gauss",
            "mag_z_gauss",

            "accel_x_mps2",
            "accel_y_mps2",
            "accel_z_mps2",

            "gyro_x_radps",
            "gyro_y_radps",
            "gyro_z_radps",

            "gps_time_s",
            "gps_week",
            "ins_status",

            "latitude_deg",
            "longitude_deg",
            "altitude_m",

            "vel_n_mps",
            "vel_e_mps",
            "vel_d_mps",

            "raw_message",
        ]

        self.zero_offsets = {
            "yaw_deg": 0.0,
            "pitch_deg": 0.0,
            "roll_deg": 0.0,
        }

        self.last_raw_orientation = None

    def find_port(self):
        ports = []

        ports += glob.glob("/dev/serial/by-id/*")
        ports += glob.glob("/dev/ttyUSB*")
        ports += glob.glob("/dev/ttyACM*")

        if not ports:
            raise RuntimeError(
                "No VN-200 serial port found.\n"
                "Try:\n"
                "ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null"
            )

        return ports[0]

    def send_command(self, command):
        if self.serial_port is None:
            return

        message = command.strip() + "\r\n"
        self.serial_port.write(message.encode("ascii"))
        self.serial_port.flush()
        time.sleep(0.1)

    def configure_output(self):
        output_hz = int(os.environ.get("VN200_OUTPUT_HZ", "25"))

        print("Configuring VN-200 output...")
        print("Output packet: VNYMR")
        print(f"Output rate:   {output_hz} Hz")

        # Register 07 = async output frequency.
        self.send_command(f"$VNWRG,07,{output_hz}*XX")

        # Register 06 = async output type.
        # 14 = VNYMR = yaw/pitch/roll + mag + accel + gyro.
        self.send_command("$VNWRG,06,14*XX")

        self.serial_port.reset_input_buffer()

    def connect(self):
        port = os.environ.get("VN200_PORT", "auto")
        baud = int(os.environ.get("VN200_BAUD", "115200"))

        if port == "auto":
            port = self.find_port()

        print(f"Connecting to VN-200 on {port} at {baud} baud...")

        self.serial_port = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
        )

        self.serial_port.reset_input_buffer()

        print("VN-200 serial connection opened.")

        self.configure_output()

    def blank_data(self):
        return {field: "" for field in self.csv_fields}

    def safe_float(self, value):
        try:
            return float(value)
        except (ValueError, TypeError):
            return ""

    def wrap_angle(self, angle):
        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle

    def has_orientation(self, data):
        return (
            isinstance(data["yaw_deg"], float)
            and isinstance(data["pitch_deg"], float)
            and isinstance(data["roll_deg"], float)
        )

    def apply_zero(self, data):
        self.last_raw_orientation = {
            "yaw_deg": data["yaw_deg"],
            "pitch_deg": data["pitch_deg"],
            "roll_deg": data["roll_deg"],
        }

        data["yaw_deg"] = self.wrap_angle(
            data["yaw_deg"] - self.zero_offsets["yaw_deg"]
        )

        data["pitch_deg"] = self.wrap_angle(
            data["pitch_deg"] - self.zero_offsets["pitch_deg"]
        )

        data["roll_deg"] = self.wrap_angle(
            data["roll_deg"] - self.zero_offsets["roll_deg"]
        )

        return data

    def parse_line(self, line):
        clean = line.strip()

        if not clean:
            return None

        if not clean.startswith("$"):
            return None

        data = self.blank_data()
        data["raw_message"] = clean

        clean = clean[1:]
        clean = clean.split("*", 1)[0]

        parts = clean.split(",")

        if len(parts) < 1:
            return None

        message_type = parts[0].strip()
        data["message_type"] = message_type

        # $VNYMR,yaw,pitch,roll,magX,magY,magZ,accelX,accelY,accelZ,gyroX,gyroY,gyroZ
        if message_type == "VNYMR" and len(parts) >= 13:
            data["yaw_deg"] = self.safe_float(parts[1])
            data["pitch_deg"] = self.safe_float(parts[2])
            data["roll_deg"] = self.safe_float(parts[3])

            data["mag_x_gauss"] = self.safe_float(parts[4])
            data["mag_y_gauss"] = self.safe_float(parts[5])
            data["mag_z_gauss"] = self.safe_float(parts[6])

            data["accel_x_mps2"] = self.safe_float(parts[7])
            data["accel_y_mps2"] = self.safe_float(parts[8])
            data["accel_z_mps2"] = self.safe_float(parts[9])

            data["gyro_x_radps"] = self.safe_float(parts[10])
            data["gyro_y_radps"] = self.safe_float(parts[11])
            data["gyro_z_radps"] = self.safe_float(parts[12])

            if self.has_orientation(data):
                return self.apply_zero(data)

            return None

        # Fallback: $VNYPR,yaw,pitch,roll
        if message_type == "VNYPR" and len(parts) >= 4:
            data["yaw_deg"] = self.safe_float(parts[1])
            data["pitch_deg"] = self.safe_float(parts[2])
            data["roll_deg"] = self.safe_float(parts[3])

            if self.has_orientation(data):
                return self.apply_zero(data)

            return None

        # Fallback: $VNYIA,yaw,pitch,roll,inertialAccelX,inertialAccelY,inertialAccelZ,gyroX,gyroY,gyroZ
        if message_type == "VNYIA" and len(parts) >= 10:
            data["yaw_deg"] = self.safe_float(parts[1])
            data["pitch_deg"] = self.safe_float(parts[2])
            data["roll_deg"] = self.safe_float(parts[3])

            data["accel_x_mps2"] = self.safe_float(parts[4])
            data["accel_y_mps2"] = self.safe_float(parts[5])
            data["accel_z_mps2"] = self.safe_float(parts[6])

            data["gyro_x_radps"] = self.safe_float(parts[7])
            data["gyro_y_radps"] = self.safe_float(parts[8])
            data["gyro_z_radps"] = self.safe_float(parts[9])

            if self.has_orientation(data):
                return self.apply_zero(data)

            return None

        # Fallback: $VNINS,time,week,status,yaw,pitch,roll,lat,lon,alt,velN,velE,velD,...
        if message_type == "VNINS" and len(parts) >= 7:
            data["gps_time_s"] = self.safe_float(parts[1])
            data["gps_week"] = parts[2]
            data["ins_status"] = parts[3]

            data["yaw_deg"] = self.safe_float(parts[4])
            data["pitch_deg"] = self.safe_float(parts[5])
            data["roll_deg"] = self.safe_float(parts[6])

            if len(parts) > 7:
                data["latitude_deg"] = self.safe_float(parts[7])

            if len(parts) > 8:
                data["longitude_deg"] = self.safe_float(parts[8])

            if len(parts) > 9:
                data["altitude_m"] = self.safe_float(parts[9])

            if len(parts) > 10:
                data["vel_n_mps"] = self.safe_float(parts[10])

            if len(parts) > 11:
                data["vel_e_mps"] = self.safe_float(parts[11])

            if len(parts) > 12:
                data["vel_d_mps"] = self.safe_float(parts[12])

            if self.has_orientation(data):
                return self.apply_zero(data)

            return None

        return None

    def read(self):
        if self.serial_port is None:
            raise RuntimeError("VN-200 is not connected yet.")

        deadline = time.monotonic() + 0.25

        while time.monotonic() < deadline:
            raw = self.serial_port.readline()

            if not raw:
                continue

            line = raw.decode("ascii", errors="replace").strip()
            data = self.parse_line(line)

            if data is not None:
                return data

        return None

    def zero(self):
        if self.last_raw_orientation is None:
            return False

        self.zero_offsets["yaw_deg"] = self.last_raw_orientation["yaw_deg"]
        self.zero_offsets["pitch_deg"] = self.last_raw_orientation["pitch_deg"]
        self.zero_offsets["roll_deg"] = self.last_raw_orientation["roll_deg"]

        return True
