import time  # importing time library to make Rpi wait because its too impatient
import pigpio  # importing GPIO library


class GPIOAdaptor:

    def __init__(self):
        self.ESC = 4  # Connect the ESC in this GPIO pin
        self.SERVO = 3
        time.sleep(1)  # As i said it is too impatient and so if this delay is removed you will get an error
        self.pi = pigpio.pi();
        self.pi.set_servo_pulsewidth(self.ESC, 0)

    def set_thrust(self, thrust):
        self.pi.set_servo_pulsewidth(self.ESC, thrust)

    def set_heading(self, heading):
        self.pi.set_servo_pulsewidth(self.SERVO, heading)
