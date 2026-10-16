"""Print readings, and say whenever the wake-up interrupt sees motion."""

import time
from machine import I2C, Pin

import lsm6dsl

i2c = I2C(0, sda=Pin(19), scl=Pin(20), freq=400_000)
imu = lsm6dsl.LSM6DSL(i2c)
imu.wake_up_interrupt(threshold_g=0.1)

while True:
    if imu.wake_up_source() & lsm6dsl.WU_IA:
        print("motion!")
    ax, ay, az = imu.acceleration()
    gx, gy, gz = imu.gyro()
    print("accel {:+.2f} {:+.2f} {:+.2f} g   gyro {:+7.1f} {:+7.1f} {:+7.1f} °/s".format(ax, ay, az, gx, gy, gz))
    time.sleep_ms(250)
