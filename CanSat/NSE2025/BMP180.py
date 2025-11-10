import pigpio # smbus2からpigpioに変更
import time
import math

class BMP180:
    """
    Bosch BMP180 Barometric Pressure and Temperature Sensor library for Raspberry Pi.
    This class handles I2C communication via pigpio.
    """

    BMP180_I2C_ADDR = 0x77

    # --- Register Map ---
    BMP180_CHIP_ID_ADDR = 0xD0
    BMP180_CONTROL_MEAS_ADDR = 0xF4
    BMP180_OUT_MSB_ADDR = 0xF6
    BMP180_CAL_AC1_ADDR = 0xAA

    # --- Commands ---
    BMP180_COMMAND_TEMP = 0x2E

    # --- OSS Settings ---
    OSS_SETTINGS = {
        0: 0.0045,
        1: 0.0075,
        2: 0.0135,
        3: 0.0255
    }
    DEFAULT_SEA_LEVEL_PA = 101325.0

    def __init__(self, pi, i2c_bus=1, i2c_address=BMP180_I2C_ADDR, oss=3):
        """
        Initializes the BMP180 object using a pigpio instance.
        :param pi: An active pigpio.pi() instance.
        :param i2c_bus: The I2C bus number.
        :param i2c_address: The I2C address of the BMP180 sensor.
        :param oss: Oversampling setting for pressure measurement (0-3).
        """
        self.pi = pi
        self.addr = i2c_address
        self.oss = oss
        self.handle = self.pi.i2c_open(i2c_bus, self.addr) # Open I2C handle

        self.cal_data = {}
        self._B5 = 0

    def _read_byte(self, reg):
        return self.pi.i2c_read_byte_data(self.handle, reg)

    def _write_byte(self, reg, value):
        self.pi.i2c_write_byte_data(self.handle, reg, value)

    def _read_signed_word_msb_first(self, reg):
        try:
            count, data = self.pi.i2c_read_i2c_block_data(self.handle, reg, 2)
            if count == 2:
                value = (data[0] << 8) | data[1] # MSB is data[0]
                if value & 0x8000:
                    value -= 0x10000
                return value
            else:
                raise IOError(f"Expected 2 bytes, but received {count}")
        except Exception as e:
            print(f"I/O error reading 16-bit word from 0x{reg:02X}: {e}")
            raise

    def _read_calibration_data(self):
        try:
            count, raw_cal_data = self.pi.i2c_read_i2c_block_data(self.handle, self.BMP180_CAL_AC1_ADDR, 22)
            if count != 22:
                 raise IOError(f"Expected 22 cal bytes, but received {count}")
            
            keys = ['AC1', 'AC2', 'AC3', 'AC4', 'AC5', 'AC6', 'B1', 'B2', 'MB', 'MC', 'MD']
            signed_keys = ['AC1', 'AC2', 'AC3', 'B1', 'B2', 'MB', 'MC', 'MD']
            
            i = 0
            for key in keys:
                self.cal_data[key] = (raw_cal_data[i] << 8) | raw_cal_data[i+1]
                i += 2

            for key in signed_keys:
                if self.cal_data[key] & 0x8000:
                    self.cal_data[key] -= 0x10000
            
            return True
        except Exception as e:
            print(f"An error occurred reading calibration data: {e}")
            return False

    def setUp(self):
        try:
            print("Attempting to set up BMP180 sensor via pigpio...")
            chip_id = self._read_byte(self.BMP180_CHIP_ID_ADDR)
            if chip_id != 0x55:
                print(f"Error: BMP180 chip ID mismatch. Expected 0x55, got 0x{chip_id:02X}")
                return False
            print(f"BMP180 Chip ID: 0x{chip_id:02X} (OK)")
            if not self._read_calibration_data():
                print("Failed to read BMP180 calibration data.")
                return False
            print("BMP180 calibration data read successfully.")
            print(f"BMP180 initialized successfully with OSS={self.oss}.")
            return True
        except Exception as e:
            print(f"An error occurred during BMP180 setup: {e}")
            return False

    def __del__(self):
        try:
            if hasattr(self, 'handle'):
                self.pi.i2c_close(self.handle) # Close I2C handle
                print('BMP180 instance deleted and handle closed.')
        except Exception as e:
            print(f"Error during BMP180 instance deletion: {e}")

    def _read_raw_temperature(self):
        self._write_byte(self.BMP180_CONTROL_MEAS_ADDR, self.BMP180_COMMAND_TEMP)
        time.sleep(0.005) # Min 4.5ms
        return self._read_signed_word_msb_first(self.BMP180_OUT_MSB_ADDR)

    def getTemperature(self):
        try:
            UT = self._read_raw_temperature()
            X1 = (UT - self.cal_data['AC6']) * self.cal_data['AC5'] / 2**15
            X2 = self.cal_data['MC'] * 2**11 / (X1 + self.cal_data['MD'])
            self._B5 = X1 + X2
            return ((self._B5 + 8) / 2**4) / 10.0
        except Exception as e:
            print(f"Error calculating temperature: {e}")
            return 0.0

    def _read_raw_pressure(self):
        command = 0x34 + (self.oss << 6)
        self._write_byte(self.BMP180_CONTROL_MEAS_ADDR, command)
        time.sleep(self.OSS_SETTINGS[self.oss])
        
        count, data = self.pi.i2c_read_i2c_block_data(self.handle, self.BMP180_OUT_MSB_ADDR, 3)
        if count == 3:
            msb, lsb, xlsb = data[0], data[1], data[2]
            return ((msb << 16) + (lsb << 8) + xlsb) >> (8 - self.oss)
        else:
            raise IOError(f"Expected 3 bytes for pressure, but received {count}")
    
    # getPressure, getAltitudeはI2C通信部分が抽象化されているため、変更不要
    # 以下、元のファイルのまま...
    def getPressure(self):
        try:
            if self._B5 == 0:
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
            if B7 < 0x80000000:
                p = (B7 * 2) / B4
            else:
                p = (B7 / B4) * 2
            X1 = (p / 2**8) * (p / 2**8)
            X1 = (X1 * 3038) / 2**16
            X2 = (-7357 * p) / 2**16
            pressure = p + (X1 + X2 + 3791) / 2**4
            return pressure
        except Exception as e:
            print(f"Error calculating pressure: {e}")
            return 0.0

    def getAltitude(self, sea_level_pressure=DEFAULT_SEA_LEVEL_PA):
        try:
            current_pressure = self.getPressure()
            if current_pressure == 0.0:
                return 0.0
            altitude = 44330.0 * (1 - math.pow(current_pressure / sea_level_pressure, 1/5.255))
            return altitude
        except Exception as e:
            print(f"Error calculating altitude: {e}")
            return 0.0

# --- Usage Example ---
if __name__ == '__main__':
    # Initialize BMP180 with Oversampling Setting (OSS=3 for highest resolution)
    sensor = BMP180(oss=3) 

    if not sensor.setUp():
        print("Failed to initialize BMP180 sensor. Exiting.")
        exit()

    print("\nBMP180 initialized successfully.")
    print("Reading temperature, pressure, and altitude...")
    print("-" * 50)
    
    try:
        while True:
            # Ensure temperature is read before pressure for accurate compensation
            temperature = sensor.getTemperature() 
            pressure_pa = sensor.getPressure()
            
            # Convert pressure to hPa/mbar for easier readability
            pressure_hpa = pressure_pa / 100.0

            # Calculate altitude using the measured pressure. 
            # You can pass your local sea level pressure here for more accuracy,
            # e.g., sensor.getAltitude(101250) if local QNH is 1012.5 hPa
            altitude = sensor.getAltitude() 

            print(f"Temperature: {temperature:.2f} °C")
            print(f"Pressure: {pressure_pa:.2f} Pa ({pressure_hpa:.2f} hPa)")
            print(f"Altitude: {altitude:.2f} m")
            print("-" * 50)
            
            time.sleep(1.0) # Read every 1 second

    except KeyboardInterrupt:
        print("\nExiting BMP180 test.")
    except Exception as e:
        print(f"An unexpected error occurred during data reading: {e}")
    finally:
        # The __del__ method will be called automatically when object is garbage collected
        pass
