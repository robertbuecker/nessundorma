#! /usr/bin/env python3
import board
import adafruit_bno055
from time import sleep, time
import numpy as np
import pprint
t0 = time()
i2c = board.I2C()
i2c.init(board.SCL_1,board.SDA_1, 800)
print(f'I2C init after {time()-t0} s')
print(i2c.scan())
sensor = adafruit_bno055.BNO055_I2C(i2c)
print(f'Sensor init after {time()-t0} s')
pp = pprint.PrettyPrinter(indent=4)

while True:
    # print(sensor.euler)
    # calib: sys, gyr, acc, mag
    sensor.mode = adafruit_bno055.NDOF_MODE
    sleep(1)
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

    t0 = time()
    print('Sensor reset')
    sensor._write_register(adafruit_bno055._TRIGGER_REGISTER, 0x20 | 
        sensor._read_register(adafruit_bno055._TRIGGER_REGISTER))
    sleep(.65)
    print(f'Sensor mode is {sensor.mode}')
    sensor.mode = adafruit_bno055.CONFIG_MODE
    sensor.offsets_accelerometer = calibdat['off_acc']
    pp.pprint(sensordat)
    pp.pprint(calibdat)
