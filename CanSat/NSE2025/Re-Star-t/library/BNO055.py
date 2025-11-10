# BNO055.py
import smbus2
import time

class BNO055:
    
    BNO055_I2C_ADDR = 0x28
    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_PAGE_ID_ADDR = 0x07
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08
    BNO055_MAG_DATA_X_LSB_ADDR = 0x0E
    BNO055_GYRO_DATA_X_LSB_ADDR = 0x14
    BNO055_EULER_H_LSB_ADDR = 0x1A
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20
    BNO055_TEMP_ADDR = 0x34
    BNO055_CALIB_STAT_ADDR = 0x35
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_PWR_MODE_ADDR = 0x3E
    BNO055_SYS_TRIGGER_ADDR = 0x3F
    BNO055_UNIT_SEL_ADDR = 0x3B
    BNO055_SYS_STAT_ADDR = 0x39
    BNO055_SYS_ERR_ADDR = 0x3A
    OPERATION_MODE_CONFIG = 0x00
    OPERATION_MODE_NDOF = 0x0C
    POWER_MODE_NORMAL = 0x00
    ACCEL_SCALE = 100.0
    GYRO_SCALE = 16.0
    MAG_SCALE = 16.0
    EULER_SCALE = 16.0
    QUAT_SCALE_FACTOR = (1.0 / (1 << 14))


    def __init__(self, i2c_bus=1, i2c_address=BNO055_I2C_ADDR):
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = i2c_address
        self._accel = [0.0, 0.0, 0.0]
        self._gyro = [0.0, 0.0, 0.0]
        self._mag = [0.0, 0.0, 0.0]
        self._euler = [0.0, 0.0, 0.0]

    def _read_byte(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def _write_byte(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def _read_signed_word(self, lsb_reg):
        try:
            data = self.bus.read_i2c_block_data(self.addr, lsb_reg, 2)
            value = (data[1] << 8) | data[0]
            if value & 0x8000:
                value -= 0x10000
            return value
        except Exception as e:
            print(f"I/O error reading 16-bit word from 0x{lsb_reg:02X}: {e}")
            raise

    def setUp(self, operation_mode=OPERATION_MODE_NDOF):
        try:
            chip_id = self._read_byte(self.BNO055_CHIP_ID_ADDR)
            if chip_id != 0xA0:
                return False
            self._set_mode(self.OPERATION_MODE_CONFIG)
            time.sleep(0.02)
            self._write_byte(self.BNO055_SYS_TRIGGER_ADDR, 0x20)
            time.sleep(0.7)
            self._write_byte(self.BNO055_PWR_MODE_ADDR, self.POWER_MODE_NORMAL)
            time.sleep(0.01)
            self._write_byte(self.BNO055_PAGE_ID_ADDR, 0x00)
            time.sleep(0.01)
            self._write_byte(self.BNO055_UNIT_SEL_ADDR, 0x00)
            time.sleep(0.01)
            self._set_mode(operation_mode)
            time.sleep(0.03)
            return True
        except Exception as e:
            return False
    def _set_mode(self, mode):
        self._write_byte(self.BNO055_OPR_MODE_ADDR, mode)
        time.sleep(0.03)
    def __del__(self):
        try:
            if hasattr(self, 'bus'):
                self._set_mode(self.OPERATION_MODE_CONFIG)
                self.bus.close()
        except Exception as e:
            pass
    def getAcc(self):
        self._accel[0] = self._read_signed_word(self.BNO055_ACCEL_DATA_X_LSB_ADDR) / self.ACCEL_SCALE
        self._accel[1] = self._read_signed_word(self.BNO055_ACCEL_DATA_X_LSB_ADDR + 2) / self.ACCEL_SCALE
        self._accel[2] = self._read_signed_word(self.BNO055_ACCEL_DATA_X_LSB_ADDR + 4) / self.ACCEL_SCALE
        return self._accel
    def getGyro(self):
        self._gyro[0] = self._read_signed_word(self.BNO055_GYRO_DATA_X_LSB_ADDR) / self.GYRO_SCALE
        self._gyro[1] = self._read_signed_word(self.BNO055_GYRO_DATA_X_LSB_ADDR + 2) / self.GYRO_SCALE
        self._gyro[2] = self._read_signed_word(self.BNO055_GYRO_DATA_X_LSB_ADDR + 4) / self.GYRO_SCALE
        return self._gyro
    def getMag(self):
        self._mag[0] = self._read_signed_word(self.BNO055_MAG_DATA_X_LSB_ADDR) / self.MAG_SCALE
        self._mag[1] = self._read_signed_word(self.BNO055_MAG_DATA_X_LSB_ADDR + 2) / self.MAG_SCALE
        self._mag[2] = self._read_signed_word(self.BNO055_MAG_DATA_X_LSB_ADDR + 4) / self.MAG_SCALE
        return self._mag
    def getEuler(self):
        self._euler[0] = self._read_signed_word(self.BNO055_EULER_H_LSB_ADDR) / self.EULER_SCALE
        self._euler[1] = self._read_signed_word(self.BNO055_EULER_H_LSB_ADDR + 2) / self.EULER_SCALE
        self._euler[2] = self._read_signed_word(self.BNO055_EULER_H_LSB_ADDR + 4) / self.EULER_SCALE
        return self._euler
    def getCalibrationStatus(self):
        cal_status = self._read_byte(self.BNO055_CALIB_STAT_ADDR)
        sys_cal = (cal_status >> 6) & 0x03
        gyro_cal = (cal_status >> 4) & 0x03
        accel_cal = (cal_status >> 2) & 0x03
        mag_cal = cal_status & 0x03
        return (sys_cal, gyro_cal, accel_cal, mag_cal)
