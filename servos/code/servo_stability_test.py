"""
Servo stability test for Raspberry Pi 5.

Wiring:
    GPIO18, physical pin 12 -> servo signal
    Pi GND, physical pin 6  -> bench supply negative
    Servo ground            -> bench supply negative
    Servo power             -> bench supply +6 V

Power supply:
    6.0 V
    3.0 A current limit
"""

from time import sleep

from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory


# ---------------------------------------------------------
# GPIO and servo configuration
# ---------------------------------------------------------

SERVO_GPIO = 18

# Conservative pulse range for initial testing.
MIN_PULSE_WIDTH = 0.0010  # 1.0 ms
MAX_PULSE_WIDTH = 0.0020  # 2.0 ms

# Standard RC servo period: 20 ms = 50 Hz.
FRAME_WIDTH = 0.020

# Only use the center portion of the servo range.
SERVO_MIN_ANGLE = -45.0
SERVO_MAX_ANGLE = 45.0

# Restrict manual testing even further.
SAFE_MIN_ANGLE = -30.0
SAFE_MAX_ANGLE = 30.0

# Ignore requested changes smaller than this amount.
COMMAND_DEADBAND_DEG = 0.75

# Move gradually to avoid sudden jumps.
MOVEMENT_STEP_DEG = 0.5
MOVEMENT_STEP_DELAY_S = 0.03


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Keep a value inside the allowed range."""
    return max(minimum, min(maximum, value))


class StableServo:
    """Servo wrapper that avoids unnecessary repeated commands."""

    def __init__(self) -> None:
        self.factory = LGPIOFactory()

        self.servo = AngularServo(
            pin=SERVO_GPIO,
            min_angle=SERVO_MIN_ANGLE,
            max_angle=SERVO_MAX_ANGLE,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            frame_width=FRAME_WIDTH,
            initial_angle=None,
            pin_factory=self.factory,
        )

        self.current_angle: float | None = None
        self.is_holding = False

    def move_to(self, requested_angle: float) -> None:
        """
        Move smoothly to a requested angle.

        Commands smaller than the deadband are ignored.
        """

        target_angle = clamp(
            requested_angle,
            SAFE_MIN_ANGLE,
            SAFE_MAX_ANGLE,
        )

        if target_angle != requested_angle:
            print(
                f"Requested angle limited to "
                f"{target_angle:+.1f} degrees."
            )

        # First command after startup.
        if self.current_angle is None:
            print(f"Moving to {target_angle:+.1f} degrees")
            self.servo.angle = target_angle
            self.current_angle = target_angle
            self.is_holding = True
            sleep(0.5)
            return

        difference = target_angle - self.current_angle

        # Do not resend nearly identical commands.
        if abs(difference) < COMMAND_DEADBAND_DEG:
            print(
                f"Ignored small change: "
                f"{self.current_angle:+.1f}° -> "
                f"{target_angle:+.1f}°"
            )
            return

        print(
            f"Moving smoothly from "
            f"{self.current_angle:+.1f}° to "
            f"{target_angle:+.1f}°"
        )

        direction = 1.0 if difference > 0 else -1.0

        while abs(target_angle - self.current_angle) > MOVEMENT_STEP_DEG:
            self.current_angle += direction * MOVEMENT_STEP_DEG
            self.servo.angle = self.current_angle
            sleep(MOVEMENT_STEP_DELAY_S)

        # Send the exact final command once.
        self.current_angle = target_angle
        self.servo.angle = target_angle
        self.is_holding = True

        print(f"Holding {self.current_angle:+.1f} degrees")

    def hold(self) -> None:
        """Resume holding the most recent angle."""

        if self.current_angle is None:
            self.current_angle = 0.0

        self.servo.angle = self.current_angle
        self.is_holding = True

        print(f"Holding {self.current_angle:+.1f} degrees")

    def release(self) -> None:
        """
        Stop sending PWM.

        The servo will no longer actively hold position.
        This is useful for diagnosing whether twitching is
        associated with the active PWM/holding loop.
        """

        self.servo.detach()
        self.is_holding = False

        print("PWM released.")
        print("The servo is no longer actively holding position.")

    def run_sweep(self) -> None:
        """Run a slow movement test."""

        for angle in [0, -10, 0, 10, 0, -20, 0, 20, 0]:
            self.move_to(float(angle))
            sleep(1.0)

    def close(self) -> None:
        """Release the servo and close GPIO resources."""

        self.servo.detach()
        self.servo.close()
        self.factory.close()


def main() -> None:
    servo = StableServo()

    try:
        print()
        print("Servo stability test")
        print("--------------------")
        print("Enter an angle between -30 and +30")
        print("s = slow automatic sweep")
        print("h = hold the current position")
        print("r = release PWM")
        print("q = quit")
        print()
        print("Starting at the center position...")

        servo.move_to(0.0)

        while True:
            command = input("\nCommand: ").strip().lower()

            if command == "q":
                break

            if command == "s":
                servo.run_sweep()
                continue

            if command == "h":
                servo.hold()
                continue

            if command == "r":
                servo.release()
                continue

            try:
                requested_angle = float(command)
            except ValueError:
                print("Enter an angle, s, h, r, or q.")
                continue

            servo.move_to(requested_angle)

    except KeyboardInterrupt:
        print("\nStopped with Ctrl+C.")

    finally:
        servo.close()
        print("PWM stopped.")
        print("Turn off the bench supply output.")


if __name__ == "__main__":
    main()
