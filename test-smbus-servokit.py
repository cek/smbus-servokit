# SPDX-License-Identifier: MIT

import unittest
from SMBusServoKit import ServoKit

# Test setup is installation-specific.
# In this case, the PCA9685 is assumed to be resident on I2C bus 7 at address 0x44.
# One servo is assumed to be attached to the controller on channel 0.
# The servo has an actuation range of 180 degrees, with min and max pulse widths as given below below
I2C_BUS = 7
I2C_ADDRESS = 0x44
MIN_PULSE_WIDTH = 400
MAX_PULSE_WIDTH = 2500

class TestServo(unittest.TestCase):
    kit = None

    def setUp(self):
        self.kit = ServoKit(channels=16, bus=I2C_BUS, address=I2C_ADDRESS)
        self.assertTrue(self.kit)

    def test_defaults(self):
        self.assertEqual(self.kit.frequency, 50)
        self.assertEqual(self.kit.servo[0].actuation_range, 180)

    def test_actuation_range(self):
        self.kit.servo[0].actuation_range = 90
        self.assertEqual(self.kit.servo[0].actuation_range, 90)
        with self.assertRaises(ValueError):
            self.kit.servo[0].angle = 91

    def test_frequency(self):
        self.kit.frequency = 60
        self.assertEqual(self.kit.frequency, 60)
        with self.assertRaises(ValueError):
            self.kit.frequency = 0

    def test_pulse_width(self):
        self.kit.servo[0].set_pulse_width_range(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
        self.assertEqual(self.kit.servo[0].min_pulse, MIN_PULSE_WIDTH)
        self.assertEqual(self.kit.servo[0].max_pulse, MAX_PULSE_WIDTH)
        with self.assertRaises(ValueError):
            self.kit.servo[0].set_pulse_width_range(MAX_PULSE_WIDTH, MIN_PULSE_WIDTH)

    def test_angle(self):
        self.kit.servo[0].angle = 42
        self.assertEqual(self.kit.servo[0].angle, 42)
        with self.assertRaises(ValueError):
            self.kit.servo[0].angle = -1
        with self.assertRaises(ValueError):
            self.kit.servo[0].angle = 200

    def test_fraction(self):
        self.kit.servo[0].fraction = 0.25
        self.assertEqual(self.kit.servo[0].fraction, 0.25)
        with self.assertRaises(ValueError):
            self.kit.servo[0].fraction = -0.5

if __name__ == '__main__':
    unittest.main()
