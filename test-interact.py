# SPDX-License-Identifier: MIT

from SMBusServoKit import ServoKit

kit = ServoKit(channels=16, bus=7, address=0x44)
kit.servo[0].set_pulse_width_range(400, 2500)

while True:
    try:
        kit.servo[0].angle = float(input("> "))
    except ValueError:
        print("Bzzzt!")
