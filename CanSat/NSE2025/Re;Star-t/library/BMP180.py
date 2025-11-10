# BMP180.py
import smbus2
import time
import math

class BMP180:
    # I2Cアドレスとレジスタ定義
    BMP180_I2C_ADDR = 0x77
    BMP180_CHIP_ID_ADDR = 0xD0
    BMP180_CONTROL_MEAS_ADDR = 0xF4
    BMP180_OUT_MSB_ADDR = 0xF6
    BMP180_CAL_AC1_ADDR = 0xAA
    BMP180_COMMAND_TEMP = 0x2E
    OSS_SETTINGS = {0: 0.0045, 1: 0.0075, 2: 0.0135, 3: 0.0255}
    DEFAULT_SEA_LEVEL_PA = 101325.0


    def __init__(self, i2c_bus=1, i2c_address=BMP180_I2C_ADDR, oss=3):
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = i2c_address
        self.oss = oss
        self.cal_data = {}
        self._B5 = 0


    def _read_byte(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def _write_byte(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def _read_signed_word_msb_first(self, reg):
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        value = (data[0] << 8) | data[1]
        if value & 0x8000:
            value -= 0x10000
        return value

    def _read_calibration_data(self):
        try:
            keys = ['AC1', 'AC2', 'AC3', 'B1', 'B2', 'MB', 'MC', 'MD']
            unsigned_keys = ['AC4', 'AC5', 'AC6']
            raw_data = self.bus.read_i2c_block_data(self.addr, self.BMP180_CAL_AC1_ADDR, 22)
            i = 0
            for key in keys:
                val = (raw_data[i] << 8) | raw_data[i+1]
                if val & 0x8000: val -= 0x10000
                self.cal_data[key] = val
                i += 2
            for key in unsigned_keys:
                self.cal_data[key] = (raw_data[i] << 8) | raw_data[i+1]
                i += 2
            return True
        except IOError:
            return False

    def setUp(self):
        if self._read_byte(self.BMP180_CHIP_ID_ADDR) != 0x55:
            return False
        return self._read_calibration_data()

    def __del__(self):
        try:
            if hasattr(self, 'bus'):
                self.bus.close()
        except:
            pass

    def _read_raw_temperature(self):
        self._write_byte(self.BMP180_CONTROL_MEAS_ADDR, self.BMP180_COMMAND_TEMP)
        time.sleep(0.005)
        return self._read_signed_word_msb_first(self.BMP180_OUT_MSB_ADDR)

    def getTemperature(self):
        UT = self._read_raw_temperature()
        X1 = (UT - self.cal_data['AC6']) * self.cal_data['AC5'] / 2**15
        X2 = self.cal_data['MC'] * 2**11 / (X1 + self.cal_data['MD'])
        self._B5 = X1 + X2
        return ((self._B5 + 8) / 2**4) / 10.0

    def _read_raw_pressure(self):
        command = 0x34 + (self.oss << 6)
        self._write_byte(self.BMP180_CONTROL_MEAS_ADDR, command)
        time.sleep(self.OSS_SETTINGS[self.oss])
        data = self.bus.read_i2c_block_data(self.addr, self.BMP180_OUT_MSB_ADDR, 3)
        return ((data[0] << 16) + (data[1] << 8) + data[2]) >> (8 - self.oss)

    def getPressure(self):
        self.getTemperature()
        UP = self._read_raw_pressure()
        B6 = self._B5 - 4000
        X1 = (self.cal_data['B2'] * (B6 * B6 / 2**12)) / 2**11
        X2 = self.cal_data['AC2'] * B6 / 2**11
        X3 = X1 + X2
        B3 = (((self.cal_data['AC1'] * 4 + X3) * (2**self.oss)) + 2) / 4
        X1 = self.cal_data['AC3'] * B6 / 2**13
        X2 = (self.cal_data['B1'] * (B6 * B6 / 2**12)) / 2**16
        X3 = ((X1 + X2) + 2) / 2**2
        B4 = self.cal_data['AC4'] * (X3 + 32768) / 2**15
        B7 = (UP - B3) * (50000 / (2**self.oss))
        p = (B7 * 2) / B4 if B7 < 0x80000000 else (B7 / B4) * 2
        X1 = (p / 2**8)**2
        X1 = (X1 * 3038) / 2**16
        X2 = (-7357 * p) / 2**16
        return p + (X1 + X2 + 3791) / 2**4

    def getAltitude(self, sea_level_pressure=DEFAULT_SEA_LEVEL_PA):
        return 44330.0 * (1 - math.pow(self.getPressure() / sea_level_pressure, 1/5.255))
