# LSM6DSL accelerometer and gyroscope

A MicroPython driver for the STMicroelectronics **LSM6DSL** 6-axis IMU
over I2C: acceleration, angular rate and die temperature, configurable
data rates and full scales, low-power modes, and the wake-up (motion)
interrupt.

Copy `lsm6dsl.py` onto your board, or install it with `mip`:

```python
import mip
mip.install("github:HowManyOliversAreThere/micropython-lsm6dsl")
```

## Usage

```python
from machine import I2C, Pin
import lsm6dsl

i2c = I2C(0, sda=Pin(19), scl=Pin(20), freq=400_000)
imu = lsm6dsl.LSM6DSL(i2c, accel_scale=2, gyro_scale=250)
print(imu.acceleration())   # (x, y, z) in g
print(imu.gyro())           # (x, y, z) in °/s
print(imu.temperature())    # °C

# Motion detection at low power: accelerometer only, interrupt on INT1.
imu.gyro_odr(lsm6dsl.ODR_OFF)
imu.accel_odr(lsm6dsl.ODR_26HZ, low_power=True)
imu.wake_up_interrupt(threshold_g=0.08)
if imu.wake_up_source() & lsm6dsl.WU_IA:   # reading clears the latched INT
    print("moved")

imu.power_down()
```

## Notes

- The address is 0x6A with SDO/SA0 low and 0x6B with it high. Tie CS
  high to select I2C.
- The constructor checks WHO_AM_I (0x6A) and raises `OSError` if it
  doesn't match, then does a software reset unless `reset=False`.
- The wake-up threshold goes in steps of full scale / 64 (31 mg at ±2 g).
- Block data update is enabled, so the bytes of one sample are never
  mixed with the next.

## License

MIT - see [LICENSE](LICENSE).
