from signal import pause

from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory


factory = LGPIOFactory()

servo = AngularServo(
    pin=18,
    min_angle=-45,
    max_angle=45,
    min_pulse_width=0.001,
    max_pulse_width=0.002,
    initial_angle=0,
    pin_factory=factory,
)

print("Holding center. Press Ctrl+C to stop.")

try:
    pause()

except KeyboardInterrupt:
    pass

finally:
    servo.detach()
    servo.close()
    factory.close()
