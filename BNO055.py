from smbus import SMBus
import time
import math

class BNO055:
    """
    Bosch BNO055 9-DOF Absolute Orientation Sensor library for Raspberry Pi.
    This class handles I2C communication and data retrieval from the BNO055.
    """

    # BNO055 I2C Address (default is 0x28, check AD0 pin for 0x29)
    BNO055_I2C_ADDR = 0x28

    # --- BNO055 Register Map (Page 0) ---
    # System Registers
    BNO055_CHIP_ID_ADDR = 0x00       # R: Chip ID, should be 0xA0
    BNO055_PAGE_ID_ADDR = 0x07       # RW: Page ID (0 for most sensor data, 1 for configuration)

    # Sensor Data Registers (Page 0) - LSB addresses for each axis
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08 # R: Accel X data (LSB)
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0A # R: Accel Y data (LSB) -- ADDED
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0C # R: Accel Z data (LSB) -- ADDED

    BNO055_MAG_DATA_X_LSB_ADDR = 0x0E    # R: Magnetometer X data (LSB)
    BNO055_MAG_DATA_Y_LSB_ADDR = 0x10    # R: Magnetometer Y data (LSB) -- ADDED
    BNO055_MAG_DATA_Z_LSB_ADDR = 0x12    # R: Magnetometer Z data (LSB) -- ADDED

    BNO055_GYRO_DATA_X_LSB_ADDR = 0x14   # R: Gyro X data (LSB)
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16   # R: Gyro Y data (LSB) -- ADDED
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18   # R: Gyro Z data (LSB) -- ADDED

    BNO055_EULER_H_LSB_ADDR = 0x1A       # R: Euler Heading data (LSB)
    BNO055_EULER_R_LSB_ADDR = 0x1C       # R: Euler Roll data (LSB) -- ADDED
    BNO055_EULER_P_LSB_ADDR = 0x1E       # R: Euler Pitch data (LSB) -- ADDED

    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20 # R: Quaternion W data (LSB)
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22 # R: Quaternion X data (LSB) -- ADDED
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24 # R: Quaternion Y data (LSB) -- ADDED
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26 # R: Quaternion Z data (LSB) -- ADDED

    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28 # R: Linear Accel X data (LSB) # Not used in example, but kept for completeness
    # ... and so on for other linear accel/gravity vectors

    BNO055_TEMP_ADDR = 0x34         # R: Temperature data
    BNO055_CALIB_STAT_ADDR = 0x35    # R: Calibration Status
    BNO055_OPR_MODE_ADDR = 0x3D       # RW: Operation Mode Register
    BNO055_PWR_MODE_ADDR = 0x3E       # RW: Power Mode Register
    BNO055_SYS_TRIGGER_ADDR = 0x3F    # RW: System Trigger Register (e.g., reset)
    BNO055_UNIT_SEL_ADDR = 0x3B       # RW: Unit Selection Register
    BNO055_SYS_STAT_ADDR = 0x39       # R: System Status Register
    BNO055_SYS_ERR_ADDR = 0x3A        # R: System Error Register

    # --- Operation Modes ---
    OPERATION_MODE_CONFIG = 0x00       # Configuration Mode
    # ... (other modes remain the same)
    OPERATION_MODE_NDOF = 0x0C         # 9-DOF Fusion (Accel + Gyro + Mag Fusion, Recommended)

    # --- Power Modes ---
    POWER_MODE_NORMAL = 0x00
    # ... (other power modes remain the same)

    # --- Scale Factors (based on default UNIT_SEL: m/s^2, dps, degrees, uT) ---
    ACCEL_SCALE = 100.0   # 1 LSB = 0.01 m/s^2
    GYRO_SCALE = 16.0     # 1 LSB = 0.0625 dps (degrees per second)
    MAG_SCALE = 16.0      # 1 LSB = 0.0625 uT (microTesla)
    EULER_SCALE = 16.0    # 1 LSB = 0.0625 degrees
    QUAT_SCALE_FACTOR = (1.0 / (1 << 14)) # 1 LSB = 1/16384 (Q14 format)


    def __init__(self, i2c_bus=1, i2c_address=BNO055_I2C_ADDR):
        """
        Initializes the BNO055 object.
        :param i2c_bus: The I2C bus number (e.g., 1 for Raspberry Pi 2/3/4).
        :param i2c_address: The I2C address of the BNO055 sensor (default 0x28).
        """
        self.i2c = SMBus(i2c_bus)
        self.addr = i2c_address

        # Initialize internal storage for sensor data
        self._accel = [0.0, 0.0, 0.0]
        self._gyro = [0.0, 0.0, 0.0]
        self._mag = [0.0, 0.0, 0.0]
        self._euler = [0.0, 0.0, 0.0] # Heading, Roll, Pitch
        self._quaternion = [0.0, 0.0, 0.0, 0.0] # W, X, Y, Z

    def _read_byte(self, reg):
        """Reads a single byte from the specified register."""
        return self.i2c.read_byte_data(self.addr, reg)

    def _write_byte(self, reg, value):
        """Writes a single byte to the specified register."""
        self.i2c.write_byte_data(self.addr, reg, value)

    def _read_signed_word(self, lsb_reg):
        """
        Reads a 16-bit signed value from two consecutive registers (LSB then MSB).
        :param lsb_reg: The address of the LSB register.
        :return: The 16-bit signed integer value.
        """
        try:
            data = self.i2c.read_i2c_block_data(self.addr,lsb_reg, 2)
            value = (data[1] << 8) | data[0]
            # lsb = self._read_byte(lsb_reg)
            # msb = self._read_byte(lsb_reg + 1)
            # value = (msb << 8) | lsb
            if value & 0x8000: # Check if negative (MSB is 1)
                value -= 0x10000 # Convert to signed 2's complement
            return value
        except IOError as e:
            print(f"I/O error reading 16-bit word from 0x{lsb_reg:02X}: {e}")
            raise # Re-raise the exception to indicate failure

    def setUp(self, operation_mode=OPERATION_MODE_NDOF):
        """
        Initializes and configures the BNO055 sensor.
        :param operation_mode: The desired BNO055 operating mode. Default is NDOF_MODE.
        :return: True if setup is successful, False otherwise.
        """
        try:
            print("Attempting to set up BNO055 sensor...")
            # 1. Check chip ID
            chip_id = self._read_byte(self.BNO055_CHIP_ID_ADDR)
            if chip_id != 0xA0:
                print(f"Error: BNO055 chip ID mismatch. Expected 0xA0, got 0x{chip_id:02X}")
                return False
            print(f"BNO055 Chip ID: 0x{chip_id:02X} (OK)")

            # 2. Set to CONFIG_MODE to allow configuration
            self._set_mode(self.OPERATION_MODE_CONFIG)
            time.sleep(0.02) # Required delay (min 19ms)

            # 3. Perform a software reset
            print("Performing BNO055 software reset.")
            self._write_byte(self.BNO055_SYS_TRIGGER_ADDR, 0x20) # S/W Reset bit
            time.sleep(0.7) # Wait for reset to complete (datasheet says 650ms max)

            # 4. Set Power Mode to Normal
            self._write_byte(self.BNO055_PWR_MODE_ADDR, self.POWER_MODE_NORMAL)
            time.sleep(0.01) # Required delay (min 7ms)

            # 5. Set Page ID to 0 (access most common registers)
            self._write_byte(self.BNO055_PAGE_ID_ADDR, 0x00)
            time.sleep(0.01)

            # 6. Set Unit Selection (default: m/s^2, dps, degrees, Celsius, Windows orientation)
            # The default value 0x00 means:
            # ACCD_UNIT=0 (m/s^2), GYR_UNIT=0 (dps), EUL_UNIT=0 (degrees), TEMP_UNIT=0 (Celsius), ORI_DEF=0 (Windows)
            self._write_byte(self.BNO055_UNIT_SEL_ADDR, 0x00)
            time.sleep(0.01)

            # 7. Set desired operation mode
            print(f"Setting operation mode to: 0x{operation_mode:02X}")
            self._set_mode(operation_mode)
            time.sleep(0.03) # Required delay (min 28ms for mode change)

            print(f"BNO055 setup complete: Mode set to 0x{operation_mode:02X}.")
            return True

        except IOError as e:
            print(f"I/O error during BNO055 setup: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred during BNO055 setup: {e}")
            return False

    def _set_mode(self, mode):
        """Helper to switch BNO055 operating mode."""
        self._write_byte(self.BNO055_OPR_MODE_ADDR, mode)
        time.sleep(0.03) # Datasheet specifies 28ms delay for mode switch

    def __del__(self):
        """
        Attempts to set the sensor back to CONFIG mode when the object is destroyed.
        This helps leave the sensor in a known state.
        """
        try:
            # Put sensor into Configuration mode before exiting
            self._set_mode(self.OPERATION_MODE_CONFIG)
            print('BNO055 instance deleted. Sensor set to CONFIG_MODE.')
        except Exception as e:
            print(f"Error during BNO055 instance deletion (likely I2C not available): {e}")

    def getAcc(self):
        """
        Reads accelerometer data (X, Y, Z) in m/s^2.
        :return: A list [x, y, z] of accelerometer values.
        """
        try:
            # Read X, Y, Z data from their respective LSB addresses and scale
            self._accel[0] = self._read_signed_word(self.BNO055_ACCEL_DATA_X_LSB_ADDR) / self.ACCEL_SCALE
            self._accel[1] = self._read_signed_word(self.BNO055_ACCEL_DATA_Y_LSB_ADDR) / self.ACCEL_SCALE
            self._accel[2] = self._read_signed_word(self.BNO055_ACCEL_DATA_Z_LSB_ADDR) / self.ACCEL_SCALE
        except IOError:
            print("Error reading accelerometer data.")
            # On error, return current (possibly stale) data. Consider raising or returning default.
        return self._accel

    def getGyro(self):
        """
        Reads gyroscope data (X, Y, Z) in degrees per second (dps).
        :return: A list [x, y, z] of gyroscope values.
        """
        try:
            # Read X, Y, Z data from their respective LSB addresses and scale
            self._gyro[0] = self._read_signed_word(self.BNO055_GYRO_DATA_X_LSB_ADDR) / self.GYRO_SCALE
            self._gyro[1] = self._read_signed_word(self.BNO055_GYRO_DATA_Y_LSB_ADDR) / self.GYRO_SCALE
            self._gyro[2] = self._read_signed_word(self.BNO055_GYRO_DATA_Z_LSB_ADDR) / self.GYRO_SCALE
        except IOError:
            print("Error reading gyroscope data.")
        return self._gyro

    def getMag(self):
        """
        Reads magnetometer data (X, Y, Z) in microTesla (uT).
        :return: A list [x, y, z] of magnetometer values.
        """
        try:
            # Read X, Y, Z data from their respective LSB addresses and scale
            self._mag[0] = self._read_signed_word(self.BNO055_MAG_DATA_X_LSB_ADDR) / self.MAG_SCALE
            self._mag[1] = self._read_signed_word(self.BNO055_MAG_DATA_Y_LSB_ADDR) / self.MAG_SCALE
            self._mag[2] = self._read_signed_word(self.BNO055_MAG_DATA_Z_LSB_ADDR) / self.MAG_SCALE
        except IOError:
            print("Error reading magnetometer data.")
        return self._mag

    def getEuler(self):
        """
        Reads Euler angle data (Heading/Yaw, Roll, Pitch) in degrees.
        This data is fusion-derived in NDOF mode.
        :return: A list [heading, roll, pitch] of Euler angles.
        """
        try:
            # Read Heading, Roll, Pitch data from their respective LSB addresses and scale
            self._euler[0] = self._read_signed_word(self.BNO055_EULER_H_LSB_ADDR) / self.EULER_SCALE
            self._euler[1] = self._read_signed_word(self.BNO055_EULER_R_LSB_ADDR) / self.EULER_SCALE
            self._euler[2] = self._read_signed_word(self.BNO055_EULER_P_LSB_ADDR) / self.EULER_SCALE
        except IOError:
            print("Error reading Euler angle data.")
        return self._euler
    
    def getQuaternion(self):
        """
        Reads quaternion data (w, x, y, z).
        This data is fusion-derived in NDOF mode.
        :return: A list [w, x, y, z] of quaternion components.
        """
        try:
            # Read W, X, Y, Z data from their respective LSB addresses and scale
            self._quaternion[0] = self._read_signed_word(self.BNO055_QUATERNION_DATA_W_LSB_ADDR) * self.QUAT_SCALE_FACTOR
            self._quaternion[1] = self._read_signed_word(self.BNO055_QUATERNION_DATA_X_LSB_ADDR) * self.QUAT_SCALE_FACTOR
            self._quaternion[2] = self._read_signed_word(self.BNO055_QUATERNION_DATA_Y_LSB_ADDR) * self.QUAT_SCALE_FACTOR
            self._quaternion[3] = self._read_signed_word(self.BNO055_QUATERNION_DATA_Z_LSB_ADDR) * self.QUAT_SCALE_FACTOR
        except IOError:
            print("Error reading quaternion data.")
        return self._quaternion

    def getTemp(self):
        """
        Reads the internal temperature of the BNO055 in Celsius.
        :return: Temperature in Celsius (°C).
        """
        try:
            temp = self._read_byte(self.BNO055_TEMP_ADDR)
            return temp
        except IOError:
            print("Error reading temperature data.")
            return 0.0 # Return a default or raise

    def getCalibrationStatus(self):
        """
        Reads the calibration status of the BNO055.
        Calibration is crucial for accurate fusion data in NDOF mode.
        :return: A tuple (system, gyro, accel, mag) where each value is 0 (uncalibrated) to 3 (fully calibrated).
        """
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
