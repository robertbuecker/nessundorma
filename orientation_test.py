#! /usr/bin/env python3
import board
import adafruit_bno055
from time import sleep
import numpy as np
import pprint
i2c = board.I2C()
i2c.init(board.SCL_1,board.SDA_1, 800)
print(i2c.scan())
sensor = adafruit_bno055.BNO055_I2C(i2c)
pp = pprint.PrettyPrinter(indent=4)
while True:
    # print(sensor.euler)
    # calib: sys, gyr, acc, mag
    sensordat = {
                        'B': sensor.magnetic,
                        'aA': tuple(x/np.pi*360 for x in sensor.gyro),
                        'aL': sensor.linear_acceleration,
                        'g': sensor.gravity,
                        'a': sensor.acceleration,
                        'calib': sensor.calibration_status}
    calibdat = {'off_acc': sensor.offsets_accelerometer,
                'off_gyr': sensor.offsets_gyroscope,
                'off_mag': sensor.offsets_magnetometer,
                'rad_mag': sensor.radius_magnetometer,
                'rad_acc': sensor.radius_accelerometer}
    sleep(0.5)
    pp.pprint(sensordat)
    pp.pprint(calibdat)
