import pigpio # smbus2からpigpioに変更
import time
import math

class BNO055:
    """
    Bosch BNO055 9-DOF Absolute Orientation Sensor library for Raspberry Pi.
    This class handles I2C communication via pigpio.
    """

    # BNO055 I2C Address (default is 0x28, check AD0 pin for 0x29)
    BNO055_I2C_ADDR = 0x28

    # --- BNO055 Register Map (Page 0) ---
    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_PAGE_ID_ADDR = 0x07
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0A
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0C
    BNO055_MAG_DATA_X_LSB_ADDR = 0x0E
    BNO055_MAG_DATA_Y_LSB_ADDR = 0x10
    BNO055_MAG_DATA_Z_LSB_ADDR = 0x12
    BNO055_GYRO_DATA_X_LSB_ADDR = 0x14
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18
    BNO055_EULER_H_LSB_ADDR = 0x1A
    BNO055_EULER_R_LSB_ADDR = 0x1C
    BNO055_EULER_P_LSB_ADDR = 0x1E
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26
    BNO055_TEMP_ADDR = 0x34
    BNO055_CALIB_STAT_ADDR = 0x35
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_PWR_MODE_ADDR = 0x3E
    BNO055_SYS_TRIGGER_ADDR = 0x3F
    BNO055_UNIT_SEL_ADDR = 0x3B
    BNO055_SYS_STAT_ADDR = 0x39
    BNO055_SYS_ERR_ADDR = 0x3A

    # --- Operation Modes ---
    OPERATION_MODE_CONFIG = 0x00
    OPERATION_MODE_NDOF = 0x0C

    # --- Power Modes ---
    POWER_MODE_NORMAL = 0x00

    # --- Scale Factors ---
    ACCEL_SCALE = 100.0
    GYRO_SCALE = 16.0
    MAG_SCALE = 16.0
    EULER_SCALE = 16.0
    QUAT_SCALE_FACTOR = (1.0 / (1 << 14))

    def __init__(self, pi, i2c_bus=1, i2c_address=BNO055_I2C_ADDR):
        """
        Initializes the BNO055 object using a pigpio instance.
        :param pi: An active pigpio.pi() instance.
        :param i2c_bus: The I2C bus number (e.g., 1 for Raspberry Pi).
        :param i2c_address: The I2C address of the BNO055 sensor.
        """
        self.pi = pi
        self.addr = i2c_address
        self.handle = self.pi.i2c_open(i2c_bus, self.addr) # Open I2C handle

        self._accel = [0.0, 0.0, 0.0]
        self._gyro = [0.0, 0.0, 0.0]
        self._mag = [0.0, 0.0, 0.0]
        self._euler = [0.0, 0.0, 0.0]
        self._quaternion = [0.0, 0.0, 0.0, 0.0]

    def _read_byte(self, reg):
        return self.pi.i2c_read_byte_data(self.handle, reg)

    def _write_byte(self, reg, value):
        self.pi.i2c_write_byte_data(self.handle, reg, value)

    def _read_signed_word(self, lsb_reg):
        try:
            count, data = self.pi.i2c_read_i2c_block_data(self.handle, lsb_reg, 2)
            if count == 2:
                value = (data[1] << 8) | data[0]
                if value & 0x8000:
                    value -= 0x10000
                return value
            else:
                raise IOError(f"Expected 2 bytes, but received {count}")
        except Exception as e:
            print(f"I/O error reading 16-bit word from 0x{lsb_reg:02X}: {e}")
            raise

    def setUp(self, operation_mode=OPERATION_MODE_NDOF):
        try:
            print("Attempting to set up BNO055 sensor via pigpio...")
            chip_id = self._read_byte(self.BNO055_CHIP_ID_ADDR)
            if chip_id != 0xA0:
                print(f"Error: BNO055 chip ID mismatch. Expected 0xA0, got 0x{chip_id:02X}")
                return False
            print(f"BNO055 Chip ID: 0x{chip_id:02X} (OK)")
            self._set_mode(self.OPERATION_MODE_CONFIG)
            time.sleep(0.02)
            print("Performing BNO055 software reset.")
            self._write_byte(self.BNO055_SYS_TRIGGER_ADDR, 0x20)
            time.sleep(0.7)
            self._write_byte(self.BNO055_PWR_MODE_ADDR, self.POWER_MODE_NORMAL)
            time.sleep(0.01)
            self._write_byte(self.BNO055_PAGE_ID_ADDR, 0x00)
            time.sleep(0.01)
            self._write_byte(self.BNO055_UNIT_SEL_ADDR, 0x00)
            time.sleep(0.01)
            print(f"Setting operation mode to: 0x{operation_mode:02X}")
            self._set_mode(operation_mode)
            time.sleep(0.03)
            print(f"BNO055 setup complete: Mode set to 0x{operation_mode:02X}.")
            return True
        except Exception as e:
            print(f"An error occurred during BNO055 setup: {e}")
            return False

    def _set_mode(self, mode):
        self._write_byte(self.BNO055_OPR_MODE_ADDR, mode)
        time.sleep(0.03)

    def __del__(self):
        try:
            if hasattr(self, 'handle'):
                self._set_mode(self.OPERATION_MODE_CONFIG)
                self.pi.i2c_close(self.handle) # Close I2C handle
                print('BNO055 instance deleted. Sensor set to CONFIG_MODE and handle closed.')
        except Exception as e:
            print(f"Error during BNO055 instance deletion: {e}")

    def getAcc(self):
        try:
            self._accel[0] = self._read_signed_word(self.BNO055_ACCEL_DATA_X_LSB_ADDR) / self.ACCEL_SCALE
            self._accel[1] = self._read_signed_word(self.BNO055_ACCEL_DATA_Y_LSB_ADDR) / self.ACCEL_SCALE
            self._accel[2] = self._read_signed_word(self.BNO055_ACCEL_DATA_Z_LSB_ADDR) / self.ACCEL_SCALE
        except IOError:
            print("Error reading accelerometer data.")
        return self._accel

    # getGyro, getMag, getEuler, getQuaternion, getTemp, getCalibrationStatus...
    # ...他のget系メソッドはI2C通信部分が抽象化されているため、変更不要です...
    def getGyro(self):
        try:
            self._gyro[0] = self._read_signed_word(self.BNO055_GYRO_DATA_X_LSB_ADDR) / self.GYRO_SCALE
            self._gyro[1] = self._read_signed_word(self.BNO055_GYRO_DATA_Y_LSB_ADDR) / self.GYRO_SCALE
            self._gyro[2] = self._read_signed_word(self.BNO055_GYRO_DATA_Z_LSB_ADDR) / self.GYRO_SCALE
        except IOError:
            print("Error reading gyroscope data.")
        return self._gyro

    def getMag(self):
        try:
            self._mag[0] = self._read_signed_word(self.BNO055_MAG_DATA_X_LSB_ADDR) / self.MAG_SCALE
            self._mag[1] = self._read_signed_word(self.BNO055_MAG_DATA_Y_LSB_ADDR) / self.MAG_SCALE
            self._mag[2] = self._read_signed_word(self.BNO055_MAG_DATA_Z_LSB_ADDR) / self.MAG_SCALE
        except IOError:
            print("Error reading magnetometer data.")
        return self._mag

    def getEuler(self):
        try:
            self._euler[0] = self._read_signed_word(self.BNO055_EULER_H_LSB_ADDR) / self.EULER_SCALE
            self._euler[1] = self._read_signed_word(self.BNO055_EULER_R_LSB_ADDR) / self.EULER_SCALE
            self._euler[2] = self._read_signed_word(self.BNO055_EULER_P_LSB_ADDR) / self.EULER_SCALE
        except IOError:
            print("Error reading Euler angle data.")
        return self._euler
    
    def getQuaternion(self):
        try:
            self._quaternion[0] = self._read_signed_word(self.BNO055_QUATERNION_DATA_W_LSB_ADDR) * self.QUAT_SCALE_FACTOR
            self._quaternion[1] = self._read_signed_word(self.BNO055_QUATERNION_DATA_X_LSB_ADDR) * self.QUAT_SCALE_FACTOR
            self._quaternion[2] = self._read_signed_word(self.BNO055_QUATERNION_DATA_Y_LSB_ADDR) * self.QUAT_SCALE_FACTOR
            self._quaternion[3] = self._read_signed_word(self.BNO055_QUATERNION_DATA_Z_LSB_ADDR) * self.QUAT_SCALE_FACTOR
        except IOError:
            print("Error reading quaternion data.")
        return self._quaternion

    def getTemp(self):
        try:
            return self._read_byte(self.BNO055_TEMP_ADDR)
        except IOError:
            print("Error reading temperature data.")
            return 0.0

    def getCalibrationStatus(self):
        try:
            cal_status = self._read_byte(self.BNO055_CALIB_STAT_ADDR)
            sys_cal = (cal_status >> 6) & 0x03
            gyro_cal = (cal_status >> 4) & 0x03
            accel_cal = (cal_status >> 2) & 0x03
            mag_cal = cal_status & 0x03
            return (sys_cal, gyro_cal, accel_cal, mag_cal)
        except IOError:
            print("Error reading calibration status.")
            return (0, 0, 0, 0)

# 他のメソッド (getSystemStatus, getSystemError) も変更不要
# 以下、元のファイルのまま...
            
    def getSystemStatus(self):
        """
        Reads the general system status code of the BNO055.
        :return: An integer representing the system status.
                 (0: Idle, 1: System Error, 2: Initializing Peripherals,
                  3: System Initialization, 4: Self Test,
                  5: Fusion Algorithm Running, 6: Sensor Fusion Running (low power))
        """
        try:
            status = self._read_byte(self.BNO055_SYS_STAT_ADDR)
            return status
        except IOError:
            print("Error reading system status byte.")
            return 0xFF # Indicate error
            
    def getSystemError(self):
        """
        Reads the system error code.
        :return: An integer representing the system error (0 = No error, non-zero = error code).
        """
        try:
            error = self._read_byte(self.BNO055_SYS_ERR_ADDR)
            return error
        except IOError:
            print("Error reading system error byte.")
            return 0xFF # Indicate error

# --- Usage Example ---
if __name__ == '__main__':
    sensor = BNO055()

    if not sensor.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("Failed to initialize BNO055 sensor. Exiting.")
        exit()

    print("\nBNO055 initialized successfully in NDOF mode.")
    print("To get accurate fusion data (Euler/Quaternion), calibrate the sensor by moving it around.")
    print("System, Gyro, Accel, Mag calibration statuses should eventually reach 3.")
    print("-" * 50)
    
    try:
        while True:
            accel = sensor.getAcc()
            gyro = sensor.getGyro()
            mag = sensor.getMag()

            euler = sensor.getEuler()
            quat = sensor.getQuaternion()

            temp = sensor.getTemp()
            cal_status = sensor.getCalibrationStatus()
            sys_status = sensor.getSystemStatus()
            sys_error = sensor.getSystemError()

            print(f"Accel (m/s^2): X={accel[0]:.2f}, Y={accel[1]:.2f}, Z={accel[2]:.2f}")
            print(f"Gyro (dps): X={gyro[0]:.2f}, Y={gyro[1]:.2f}, Z={gyro[2]:.2f}")
            print(f"Mag (uT): X={mag[0]:.2f}, Y={mag[1]:.2f}, Z={mag[2]:.2f}")
            print(f"Euler (deg): Heading(Yaw)={euler[0]:.2f}, Roll={euler[1]:.2f}, Pitch={euler[2]:.2f}")
            print(f"Quaternion: W={quat[0]:.4f}, X={quat[1]:.4f}, Y={quat[2]:.4f}, Z={quat[3]:.4f}")
            print(f"Temperature (C): {temp:.1f}")
            print(f"Calibration Status (Sys, Gyro, Accel, Mag): {cal_status}")
            print(f"System Status: {sys_status}, System Error: {sys_error}")
            print("-" * 50)
            
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting BNO055 test.")
    except Exception as e:
        print(f"An error occurred during data reading: {e}")
    finally:
        pass
