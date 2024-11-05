# -*- coding: utf-8 -*-
from smbus import SMBus
import time
import math

class BMX055:
    # I2C
    global ACCL_ADDR
    global ACCL_R_ADDR
    global GYRO_ADDR
    global GYRO_R_ADDR
    global MAG_ADDR
    global MAG_R_ADDR
    
    ACCL_ADDR = 0x19
    ACCL_R_ADDR = 0x02
    GYRO_ADDR = 0x69
    GYRO_R_ADDR = 0x02
    MAG_ADDR = 0x13
    MAG_R_ADDR = 0x42
    
    def setUp(self):

        self.i2c = SMBus(1)
        
        # acc_data_setup : 加速度の値をセットアップ
        self.i2c.write_byte_data(ACCL_ADDR, 0x0F, 0x03)
        self.i2c.write_byte_data(ACCL_ADDR, 0x10, 0x08)
        self.i2c.write_byte_data(ACCL_ADDR, 0x11, 0x00)
        time.sleep(0.5)

        # gyr_data_setup : ジャイロ値をセットアップ
        self.i2c.write_byte_data(GYRO_ADDR, 0x0F, 0x04)
        self.i2c.write_byte_data(GYRO_ADDR, 0x10, 0x07)
        self.i2c.write_byte_data(GYRO_ADDR, 0x11, 0x00)
        time.sleep(0.5)

        # mag_data_setup : 地磁気値をセットアップ
        data = self.i2c.read_byte_data(MAG_ADDR, 0x4B)
        if(data == 0):
            self.i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
            time.sleep(0.5)
        self.i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
        self.i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
        self.i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
        self.i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
        self.i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
        time.sleep(0.5)
        
        self.acc = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        
    def __del__(self):
        print('delete instance from BMX055')


    def getAcc(self):
        data = [0, 0, 0, 0, 0, 0]

        try:
            for i in range(6):
                data[i] = self.i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i)

            for i in range(3):
                self.acc[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16
                if self.acc[i] > 2047:
                    self.acc[i] -= 4096
                self.acc[i] *= 0.0098

        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))

        return self.acc

    def getGyro(self):
        data = [0, 0, 0, 0, 0, 0]

        try:
            for i in range(6):
                data[i] = self.i2c.read_byte_data(GYRO_ADDR, GYRO_R_ADDR + i)

            for i in range(3):
                self.gyro[i] = (data[2*i + 1] * 256) + data[2*i]
                if self.gyro[i] > 32767:
                    self.gyro[i] -= 65536
                self.gyro[i] *= 0.0038

        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))

        return self.gyro

    def getMag(self):
        data = [0, 0, 0, 0, 0, 0, 0, 0]

        try:
            for i in range(8):
                data[i] = self.i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)

            for i in range(3):
                if i != 2:
                    self.mag[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                    if self.mag[i] > 4095:
                        self.mag[i] -= 8192
                else:
                    self.mag[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                    if self.mag[i] > 16383:
                        self.mag[i] -= 32768

        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))

        return self.mag