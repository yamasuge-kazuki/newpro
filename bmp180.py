from smbus import SMBus
import time
import math # Altitude calculation needs math.pow

class BMP180:
    """
    Bosch BMP180 Barometric Pressure and Temperature Sensor library for Raspberry Pi.
    This class handles I2C communication, calibration data retrieval,
    and calculation of temperature, pressure, and altitude.
    """

    # BMP180 I2C Address
    BMP180_I2C_ADDR = 0x77

    # --- BMP180 Register Map ---
    BMP180_CHIP_ID_ADDR = 0xD0      # R: Chip ID, should be 0x55
    BMP180_CONTROL_MEAS_ADDR = 0xF4 # RW: Control Measurement Register
    BMP180_OUT_MSB_ADDR = 0xF6      # R: MSB of output data
    BMP180_OUT_LSB_ADDR = 0xF7      # R: LSB of output data
    BMP180_OUT_XLSB_ADDR = 0xF8     # R: XLSB of output data (for pressure)

    # Calibration Data Register Addresses (EEPROM)
    # These are 16-bit signed values
    BMP180_CAL_AC1_ADDR = 0xAA
    BMP180_CAL_AC2_ADDR = 0xAC
    BMP180_CAL_AC3_ADDR = 0xAE
    BMP180_CAL_AC4_ADDR = 0xB0
    BMP180_CAL_AC5_ADDR = 0xB2
    BMP180_CAL_AC6_ADDR = 0xB4
    BMP180_CAL_B1_ADDR = 0xB6
    BMP180_CAL_B2_ADDR = 0xB8
    BMP180_CAL_MB_ADDR = 0xBA
    BMP180_CAL_MC_ADDR = 0xBC
    BMP180_CAL_MD_ADDR = 0xBE

    # Control Measurement Command values
    BMP180_COMMAND_TEMP = 0x2E  # Read temperature (UT)
    # Pressure commands vary with Oversampling Setting (OSS)
    # BMP180_COMMAND_PRESSURE_OSS0 = 0x34
    # BMP180_COMMAND_PRESSURE_OSS1 = 0x34 + (1 << 6) # 0x74
    # BMP180_COMMAND_PRESSURE_OSS2 = 0x34 + (2 << 6) # 0xB4
    # BMP180_COMMAND_PRESSURE_OSS3 = 0x34 + (3 << 6) # 0xF4

    # Oversampling Settings (OSS) and their delays (in seconds)
    # Defines resolution and conversion time for pressure measurement
    OSS_SETTINGS = {
        0: 0.0045, # Ultra low power (4.5ms)
        1: 0.0075, # Standard (7.5ms)
        2: 0.0135, # High resolution (13.5ms)
        3: 0.0255  # Ultra high resolution (25.5ms)
    }

    # Default sea level pressure for altitude calculation (PASCAL)
    # This is a standard atmospheric pressure at sea level (1013.25 hPa)
    DEFAULT_SEA_LEVEL_PA = 101325.0

    def __init__(self, i2c_bus=1, i2c_address=BMP180_I2C_ADDR, oss=3):
        """
        Initializes the BMP180 object.
        :param i2c_bus: The I2C bus number (e.g., 1 for Raspberry Pi 2/3/4).
        :param i2c_address: The I2C address of the BMP180 sensor (default 0x77).
        :param oss: Oversampling setting for pressure measurement (0-3).
                    Higher OSS gives higher resolution but slower conversion.
        """
        self.i2c = SMBus(i2c_bus)
        self.addr = i2c_address
        self.oss = oss # Store chosen OSS

        # Internal storage for calibration data
        self.cal_data = {}
        # Values computed during temperature/pressure calculation
        self._B5 = 0 # This value is used in both temperature and pressure calculation

    def _read_byte(self, reg):
        """Reads a single byte from the specified register."""
        return self.i2c.read_byte_data(self.addr, reg)

    def _write_byte(self, reg, value):
        """Writes a single byte to the specified register."""
        self.i2c.write_byte_data(self.addr, reg, value)

    def _read_signed_word(self, lsb_reg):
        """
        Reads a 16-bit signed value from two consecutive registers (MSB then LSB).
        BMP180 calibration data is MSB first.
        :param msb_reg: The address of the MSB register.
        :return: The 16-bit signed integer value.
        """
        try:
            msb = self._read_byte(lsb_reg) # Corrected to read MSB first based on BMP180
            lsb = self._read_byte(lsb_reg + 1)
            value = (msb << 8) | lsb
            if value & 0x8000: # Check if negative (MSB is 1)
               value -= 0x10000 # Convert to signed 2's complement
            return value
            #data = self.i2c.read_i2c_block_data(self.addr, lsb_reg, 2)
            #value = (data[1] << 8) | data[0]
            #if value & 0x8000:
            #   value -= 0x10000
            #return value
        except IOError as e:
            print(f"I/O error reading 16-bit word from 0x{lsb_reg:02X}: {e}")
            raise # Re-raise the exception to indicate failure

    def _read_calibration_data(self):
        """
        Reads all 11 calibration coefficients from the BMP180's EEPROM.
        These are necessary for accurate temperature and pressure calculations.
        """
        try:
            # Using SMBus's read_i2c_block_data for efficiency
            # The calibration data spans 22 bytes from 0xAA to 0xBF
            raw_cal_data = self.i2c.read_i2c_block_data(self.addr, self.BMP180_CAL_AC1_ADDR, 22)

            # Parse each 2-byte (16-bit signed) calibration value
            self.cal_data['AC1'] = (raw_cal_data[0] << 8) | raw_cal_data[1]
            self.cal_data['AC2'] = (raw_cal_data[2] << 8) | raw_cal_data[3]
            self.cal_data['AC3'] = (raw_cal_data[4] << 8) | raw_cal_data[5]
            self.cal_data['AC4'] = (raw_cal_data[6] << 8) | raw_cal_data[7]
            self.cal_data['AC5'] = (raw_cal_data[8] << 8) | raw_cal_data[9]
            self.cal_data['AC6'] = (raw_cal_data[10] << 8) | raw_cal_data[11]
            self.cal_data['B1']  = (raw_cal_data[12] << 8) | raw_cal_data[13]
            self.cal_data['B2']  = (raw_cal_data[14] << 8) | raw_cal_data[15]
            self.cal_data['MB']  = (raw_cal_data[16] << 8) | raw_cal_data[17]
            self.cal_data['MC']  = (raw_cal_data[18] << 8) | raw_cal_data[19]
            self.cal_data['MD']  = (raw_cal_data[20] << 8) | raw_cal_data[21]
            
            # Apply 2's complement for signed values
            for key in ['AC1', 'AC2', 'AC3', 'B1', 'B2', 'MB', 'MC', 'MD']:
                if self.cal_data[key] & 0x8000:
                    self.cal_data[key] -= 0x10000

            # AC4, AC5, AC6 are unsigned but stored as 16-bit
            if self.cal_data['AC4'] < 0: self.cal_data['AC4'] += 0x10000
            if self.cal_data['AC5'] < 0: self.cal_data['AC5'] += 0x10000
            if self.cal_data['AC6'] < 0: self.cal_data['AC6'] += 0x10000


            return True
        except IOError as e:
            print(f"I/O error reading BMP180 calibration data: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred reading calibration data: {e}")
            return False

    def setUp(self):
        """
        Initializes and configures the BMP180 sensor.
        :return: True if setup is successful, False otherwise.
        """
        try:
            print("Attempting to set up BMP180 sensor...")

            # 1. Check chip ID
            chip_id = self._read_byte(self.BMP180_CHIP_ID_ADDR)
            if chip_id != 0x55:
                print(f"Error: BMP180 chip ID mismatch. Expected 0x55, got 0x{chip_id:02X}")
                return False
            print(f"BMP180 Chip ID: 0x{chip_id:02X} (OK)")

            # 2. Read calibration data
            if not self._read_calibration_data():
                print("Failed to read BMP180 calibration data.")
                return False
            print("BMP180 calibration data read successfully.")

            print(f"BMP180 initialized successfully with OSS={self.oss}.")
            return True

        except IOError as e:
            print(f"I/O error during BMP180 setup: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred during BMP180 setup: {e}")
            return False

    def __del__(self):
        """
        No specific cleanup for BMP180 needed, but good practice to have a placeholder.
        """
        print('BMP180 instance deleted.')

    def _read_raw_temperature(self):
        """
        Initiates temperature measurement and reads the uncompensated temperature value (UT).
        """
        try:
            # Write command to start temperature measurement
            self._write_byte(self.BMP180_CONTROL_MEAS_ADDR, self.BMP180_COMMAND_TEMP)
            time.sleep(self.OSS_SETTINGS[0]) # Temperature conversion time (4.5ms for OSS=0)
            
            # Read uncompensated temperature (UT)
            ut = self._read_signed_word(self.BMP180_OUT_MSB_ADDR)
            return ut
        except IOError as e:
            print(f"Error reading raw temperature: {e}")
            raise

    def getTemperature(self):
        """
        Reads and calculates the compensated temperature in Celsius.
        Follows Bosch BMP180 datasheet Section 3.3.
        :return: Temperature in Celsius (°C).
        """
        try:
            UT = self._read_raw_temperature()
            
            # Compensation calculation
            AC6 = float(self.cal_data['AC6'])
            AC5 = float(self.cal_data['AC5'])
            MC = float(self.cal_data['MC'])
            MD = float(self.cal_data['MD'])

            X1 = (UT - AC6) * AC5 / 2**15
            X2 = MC * 2**11 / (X1 + MD)
            self._B5 = X1 + X2 # Store _B5 for pressure calculation
            
            temperature = ((self._B5 + 8) / 2**4) / 10.0 # Result in 0.1 deg C, so divide by 10.0
            return temperature
        except Exception as e:
            print(f"Error calculating temperature: {e}")
            return 0.0 # Return default on error

    def _read_raw_pressure(self):
        """
        Initiates pressure measurement and reads the uncompensated pressure value (UP).
        """
        try:
            # Write command to start pressure measurement with selected OSS
            command_pressure = 0x34 + (self.oss << 6)
            self._write_byte(self.BMP180_CONTROL_MEAS_ADDR, command_pressure)
            time.sleep(self.OSS_SETTINGS[self.oss]) # Wait for conversion based on OSS

            # Read uncompensated pressure (UP), 3 bytes
            msb = self._read_byte(self.BMP180_OUT_MSB_ADDR)
            lsb = self._read_byte(self.BMP180_OUT_LSB_ADDR)
            xlsb = self._read_byte(self.BMP180_OUT_XLSB_ADDR)
            
            # Combine bytes and shift according to OSS (Datasheet P.14)
            up = ((msb << 16) | (lsb << 8) | xlsb) >> (8 - self.oss)
            return up
        except IOError as e:
            print(f"Error reading raw pressure: {e}")
            raise

    def getPressure(self):
        """
        Reads and calculates the compensated pressure in Pascals.
        Follows Bosch BMP180 datasheet Section 3.4.
        Requires getTemperature() to be called first to update self._B5.
        :return: Pressure in Pascals (Pa).
        """
        try:
            if self._B5 == 0: # Ensure temperature was read first to compute _B5
                print("Warning: Call getTemperature() before getPressure() for accurate results.")
                self.getTemperature() # Re-calculate temperature if not done

            UP = self._read_raw_pressure()

            # Compensation calculation
            AC1 = float(self.cal_data['AC1'])
            AC2 = float(self.cal_data['AC2'])
            AC3 = float(self.cal_data['AC3'])
            AC4 = float(self.cal_data['AC4'])
            B1 = float(self.cal_data['B1'])
            B2 = float(self.cal_data['B2'])

            B6 = self._B5 - 4000
            X1 = (B2 * (B6 * B6 / 2**12)) / 2**11
            X2 = AC2 * B6 / 2**11
            X3 = X1 + X2
            B3 = (((AC1 * 4 + X3) * (2**self.oss)) + 2) / 4

            X1 = AC3 * B6 / 2**13
            X2 = (B1 * (B6 * B6 / 2**12)) / 2**16
            X3 = ((X1 + X2) + 2) / 2**2
            B4 = AC4 * (X3 + 32768) / 2**15

            B7 = (UP - B3) * (50000 / (2**self.oss))
            
            if B7 < 0x80000000: # Python handles large integers, but datasheet logic
                p = (B7 * 2) / B4
            else:
                p = (B7 / B4) * 2

            X1 = (p / 2**8) * (p / 2**8)
            X1 = (X1 * 3038) / 2**16
            X2 = (-7357 * p) / 2**16
            
            pressure = p + ((X1 + X2 + 3791) / 2**4)
            
            return pressure
        except Exception as e:
            print(f"Error calculating pressure: {e}")
            return 0.0 # Return default on error

    def getAltitude(self, sea_level_pressure=DEFAULT_SEA_LEVEL_PA):
        """
        Calculates altitude in meters based on current pressure and a reference
        sea-level pressure. Uses the standard atmosphere formula.
        :param sea_level_pressure: Reference pressure at sea level in Pascals.
                                   Use 101325 Pa for standard, or your local QNH.
        :return: Altitude in meters.
        """
        try:
            current_pressure = self.getPressure()
            
            if current_pressure == 0.0: # If pressure reading failed
                return 0.0

            # Formula: h = 44330 * [1 - (P/P0)^(1/5.255)]
            altitude = 44330.0 * (1 - math.pow(current_pressure / sea_level_pressure, 1/5.255))
            return altitude
        except Exception as e:
            print(f"Error calculating altitude: {e}")
            return 0.0 # Return default on error

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
