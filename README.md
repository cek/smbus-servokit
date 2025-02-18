SMBusServoKit
=============

SMBusServoKit is a simple python module for controlling servos connected to a PCA9685 controller over I2C via smbus2.

SMBusServoKit is inspired by [adafruit-circuitpython-servokit](https://docs.circuitpython.org/projects/servokit/en/latest/), but uses a more direct communication scheme, which can be beneficial on systems that are not fully supported by CircuitPython.

This module was initially developed to support the PCA9685 on the [Jetson Orin Nano Super](https://en.wikipedia.org/wiki/Nvidia_Jetson) upon its initial release.
