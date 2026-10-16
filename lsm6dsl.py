"""
MicroPython driver for the STMicroelectronics LSM6DSL 6-axis IMU
(3-axis accelerometer + 3-axis gyroscope), over I2C.

I2C address (7-bit): 0x6A with SDO/SA0 low, 0x6B with it high. The CS
pin must be tied high for I2C.

Quick start
-----------
    from machine import I2C, Pin
    import lsm6dsl

    i2c = I2C(0, sda=Pin(19), scl=Pin(20), freq=400_000)
    imu = lsm6dsl.LSM6DSL(i2c)                   # 0x6A, accel+gyro at 104 Hz
    print(imu.acceleration())                    # (x, y, z) in g
    print(imu.gyro())                            # (x, y, z) in degrees/s

    # Wake-up (motion) interrupt on INT1, then low power while idle:
    imu.gyro_odr(lsm6dsl.ODR_OFF)
    imu.accel_odr(lsm6dsl.ODR_26HZ, low_power=True)
    imu.wake_up_interrupt(threshold_g=0.1)
    ...
    if imu.wake_up_source() & lsm6dsl.WU_IA:     # reading clears a latched INT
        print("moved")

    imu.power_down()                             # both sensors off (~3 uA)
"""

import time
from micropython import const

ADDR_LOW = 0x6A
ADDR_HIGH = 0x6B
WHO_AM_I = 0x6A

# Output data rates (accelerometer and gyroscope share the codes, except
# that 1.6 Hz is accelerometer-only and needs low-power mode).
ODR_OFF = 0
ODR_12HZ5 = 1
ODR_26HZ = 2
ODR_52HZ = 3
ODR_104HZ = 4
ODR_208HZ = 5
ODR_416HZ = 6
ODR_833HZ = 7
ODR_1660HZ = 8
ODR_3330HZ = 9
ODR_6660HZ = 10
ODR_1HZ6 = 11

# Accelerometer full scale (g) -> FS_XL code, and mg per LSB.
_ACCEL_FS = {2: (0, 0.061), 16: (1, 0.488), 4: (2, 0.122), 8: (3, 0.244)}
# Gyroscope full scale (dps) -> FS_G code (incl. FS_125), and mdps per LSB.
_GYRO_FS = {125: (1, 4.375), 250: (0, 8.75), 500: (2, 17.5), 1000: (4, 35.0), 2000: (6, 70.0)}

# WAKE_UP_SRC bits.
Z_WU = 0x01
Y_WU = 0x02
X_WU = 0x04
WU_IA = 0x08  # wake-up event
SLEEP_STATE_IA = 0x10
FF_IA = 0x20  # free fall

_REG_WHO_AM_I = const(0x0F)
_REG_CTRL1_XL = const(0x10)
_REG_CTRL2_G = const(0x11)
_REG_CTRL3_C = const(0x12)
_REG_CTRL6_C = const(0x15)
_REG_CTRL7_G = const(0x16)
_REG_WAKE_UP_SRC = const(0x1B)
_REG_D6D_SRC = const(0x1D)
_REG_STATUS = const(0x1E)
_REG_OUT_TEMP_L = const(0x20)
_REG_OUTX_L_G = const(0x22)
_REG_OUTX_L_XL = const(0x28)
_REG_TAP_CFG = const(0x58)
_REG_WAKE_UP_THS = const(0x5B)
_REG_WAKE_UP_DUR = const(0x5C)
_REG_MD1_CFG = const(0x5E)
_REG_MD2_CFG = const(0x5F)

_CTRL3_BDU = const(0x40)
_CTRL3_IF_INC = const(0x04)
_CTRL3_SW_RESET = const(0x01)
_CTRL3_H_LACTIVE = const(0x20)
_CTRL6_XL_HM_MODE = const(0x10)  # 1 = high-performance off (low power)
_CTRL7_G_HM_MODE = const(0x80)
_TAP_INTERRUPTS_ENABLE = const(0x80)
_TAP_LIR = const(0x01)
_MD_WU = const(0x20)


class LSM6DSL:
    def __init__(self, i2c, address=ADDR_LOW, accel_odr=ODR_104HZ, accel_scale=2,
                 gyro_odr=ODR_104HZ, gyro_scale=250, reset=True):
        """
        i2c         : machine.I2C (or SoftI2C) instance
        accel_scale : 2, 4, 8 or 16 g
        gyro_scale  : 125, 250, 500, 1000 or 2000 degrees/s
        reset       : software reset first, so no previous configuration
                      (e.g. interrupts) is left behind
        Raises OSError if WHO_AM_I doesn't match.
        """
        self._i2c = i2c
        self._addr = address
        self._buf6 = bytearray(6)
        self._buf2 = bytearray(2)
        ident = self._read(_REG_WHO_AM_I)
        if ident != WHO_AM_I:
            raise OSError("LSM6DSL not found (WHO_AM_I = 0x%02X)" % ident)
        if reset:
            self._write(_REG_CTRL3_C, _CTRL3_SW_RESET)
            time.sleep_ms(5)
        # Block data update (no torn reads) and register auto-increment.
        self._write(_REG_CTRL3_C, _CTRL3_BDU | _CTRL3_IF_INC)
        self._accel_fs = accel_scale
        self._gyro_fs = gyro_scale
        self._accel_odr = ODR_OFF
        self._gyro_odr = ODR_OFF
        self.accel_odr(accel_odr)
        self.gyro_odr(gyro_odr)

    # ---- Low-level I2C ----------------------------------------------------
    def _read(self, reg):
        return self._i2c.readfrom_mem(self._addr, reg, 1)[0]

    def _write(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, bytes((value & 0xFF,)))

    def _update(self, reg, mask, value):
        self._write(reg, (self._read(reg) & ~mask) | (value & mask))

    def _vector(self, reg, scale):
        self._i2c.readfrom_mem_into(self._addr, reg, self._buf6)
        b = self._buf6
        out = []
        for i in (0, 2, 4):
            v = b[i] | (b[i + 1] << 8)
            if v & 0x8000:
                v -= 0x10000
            out.append(v * scale)
        return tuple(out)

    # ---- Configuration ----------------------------------------------------
    def accel_odr(self, odr=None, scale=None, low_power=False):
        """Get or set the accelerometer data rate (ODR_*). low_power turns
        off high-performance mode, which is needed for ODR_1HZ6 and saves
        current at the lower rates."""
        if odr is None:
            return self._accel_odr
        if scale is not None:
            self._accel_fs = scale
        code = _ACCEL_FS[self._accel_fs][0]
        self._update(_REG_CTRL6_C, _CTRL6_XL_HM_MODE, _CTRL6_XL_HM_MODE if low_power else 0)
        self._write(_REG_CTRL1_XL, (odr << 4) | (code << 2))
        self._accel_odr = odr
        return odr

    def gyro_odr(self, odr=None, scale=None, low_power=False):
        """Get or set the gyroscope data rate (ODR_*, not ODR_1HZ6)."""
        if odr is None:
            return self._gyro_odr
        if odr == ODR_1HZ6:
            raise ValueError("1.6 Hz is accelerometer-only")
        if scale is not None:
            self._gyro_fs = scale
        code = _GYRO_FS[self._gyro_fs][0]
        self._update(_REG_CTRL7_G, _CTRL7_G_HM_MODE, _CTRL7_G_HM_MODE if low_power else 0)
        self._write(_REG_CTRL2_G, (odr << 4) | (code << 1))
        self._gyro_odr = odr
        return odr

    def power_down(self):
        """Both sensors off. Interrupt configuration is kept."""
        self.gyro_odr(ODR_OFF)
        self.accel_odr(ODR_OFF)

    # ---- Readings ---------------------------------------------------------
    def acceleration(self):
        """(x, y, z) in g."""
        return self._vector(_REG_OUTX_L_XL, _ACCEL_FS[self._accel_fs][1] / 1000)

    def gyro(self):
        """(x, y, z) in degrees per second."""
        return self._vector(_REG_OUTX_L_G, _GYRO_FS[self._gyro_fs][1] / 1000)

    def temperature(self):
        """Die temperature, degrees C (only updates while a sensor is on)."""
        self._i2c.readfrom_mem_into(self._addr, _REG_OUT_TEMP_L, self._buf2)
        v = self._buf2[0] | (self._buf2[1] << 8)
        if v & 0x8000:
            v -= 0x10000
        return v / 256 + 25

    def status(self):
        """STATUS_REG: bit0 accel data ready, bit1 gyro, bit2 temperature."""
        return self._read(_REG_STATUS)

    # ---- Interrupts -------------------------------------------------------
    def interrupt_pin(self, active_high=True):
        """INT1/INT2 polarity (both pins share it)."""
        self._update(_REG_CTRL3_C, _CTRL3_H_LACTIVE, 0 if active_high else _CTRL3_H_LACTIVE)

    def wake_up_interrupt(self, threshold_g=0.1, duration=0, pin=1, latched=True):
        """Raise INT1 (or INT2) when any axis' acceleration, with gravity
        filtered out, exceeds threshold_g for `duration` samples (0-3).
        With latched=True the pin stays asserted until wake_up_source()
        is read. The accelerometer must be running."""
        lsb = self._accel_fs / 64  # WK_THS is in FS_XL / 2^6 steps
        ths = max(1, min(63, int(round(threshold_g / lsb))))
        self._update(_REG_WAKE_UP_THS, 0x3F, ths)
        self._update(_REG_WAKE_UP_DUR, 0x60, (duration & 0x03) << 5)
        self._update(_REG_TAP_CFG, _TAP_INTERRUPTS_ENABLE | _TAP_LIR,
                     _TAP_INTERRUPTS_ENABLE | (_TAP_LIR if latched else 0))
        reg = _REG_MD1_CFG if pin == 1 else _REG_MD2_CFG
        self._update(reg, _MD_WU, _MD_WU)

    def disable_wake_up_interrupt(self):
        self._update(_REG_MD1_CFG, _MD_WU, 0)
        self._update(_REG_MD2_CFG, _MD_WU, 0)

    def wake_up_source(self):
        """WAKE_UP_SRC (see WU_IA etc). Reading it clears a latched
        interrupt."""
        return self._read(_REG_WAKE_UP_SRC)

    def orientation_source(self):
        """D6D_SRC: the 6D orientation detection result."""
        return self._read(_REG_D6D_SRC)
