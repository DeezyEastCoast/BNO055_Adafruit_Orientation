using System;
using Microsoft.SPOT;
using System.Threading;

namespace BNO_055_Adafurit_Orientation_Sensor
{
/***************************************************************************
This is a library for the BNO055 orientation sensor

Designed specifically to work with the Adafruit BNO055 Breakout.

Pick one up today in the adafruit shop!
------> http://www.adafruit.com/products

These sensors use I2C to communicate, 2 pins are required to interface.

Adafruit invests time and resources providing this open source code,
please support Adafruit andopen-source hardware by purchasing products
from Adafruit!

Written by KTOWN for an arduino by Adafruit Industries but it was ported to C# by your boy.

MIT license, all text above must be included in any redistribution
***************************************************************************/

    class BNO055 : SensorBase
    {
        const byte BNO055_ADDRESS_A = 0x28;
        const byte BNO055_ADDRESS_B = 0x29;
        const byte BNO055_ID = 0xA0;
        const byte NUM_BNO055_OFFSET_REGISTERS = 22;

        byte adafruit_bno055_opmode_t_mode;
        adafruit_bno055_offsets_t BNO_SENSOR_OFFSETS;

        public struct adafruit_bno055_offsets_t
        {
            public UInt16 accel_offset_x;
            public UInt16 accel_offset_y;
            public UInt16 accel_offset_z;
            public UInt16 gyro_offset_x;
            public UInt16 gyro_offset_y;
            public UInt16 gyro_offset_z;
            public UInt16 mag_offset_x;
            public UInt16 mag_offset_y;
            public UInt16 mag_offset_z;

            public UInt16 accel_radius;
            public UInt16 mag_radius;
        }

        public struct Vector
        {
            public double x;
            public double y;
            public double z;
        }

        public enum adafruit_vector_type_t
        {
            VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
            VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
            VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
            VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
            VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
            VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
        }

        #region adafruit_bno055_reg_t

        /* Page id register definition */
        const byte BNO055_PAGE_ID_ADDR = 0X07;

        /* PAGE0 REGISTER DEFINITION START*/
        const byte BNO055_CHIP_ID_ADDR = 0x00;
        const byte BNO055_ACCEL_REV_ID_ADDR = 0x01;
        const byte BNO055_MAG_REV_ID_ADDR = 0x02;
        const byte BNO055_GYRO_REV_ID_ADDR = 0x03;
        const byte BNO055_SW_REV_ID_LSB_ADDR = 0x04;
        const byte BNO055_SW_REV_ID_MSB_ADDR = 0x05;
        const byte BNO055_BL_REV_ID_ADDR = 0X06;

        /* Accel data register */
        const byte BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08;
        const byte BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09;
        const byte BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A;
        const byte BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B;
        const byte BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C;
        const byte BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D;

        /* Mag data register */
        const byte BNO055_MAG_DATA_X_LSB_ADDR = 0X0E;
        const byte BNO055_MAG_DATA_X_MSB_ADDR = 0X0F;
        const byte BNO055_MAG_DATA_Y_LSB_ADDR = 0X10;
        const byte BNO055_MAG_DATA_Y_MSB_ADDR = 0X11;
        const byte BNO055_MAG_DATA_Z_LSB_ADDR = 0X12;
        const byte BNO055_MAG_DATA_Z_MSB_ADDR = 0X13;

        /* Gyro data registers */
        const byte BNO055_GYRO_DATA_X_LSB_ADDR = 0X14;
        const byte BNO055_GYRO_DATA_X_MSB_ADDR = 0X15;
        const byte BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16;
        const byte BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17;
        const byte BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18;
        const byte BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19;

        /* Euler data registers */
        const byte BNO055_EULER_H_LSB_ADDR = 0X1A;
        const byte BNO055_EULER_H_MSB_ADDR = 0X1B;
        const byte BNO055_EULER_R_LSB_ADDR = 0X1C;
        const byte BNO055_EULER_R_MSB_ADDR = 0X1D;
        const byte BNO055_EULER_P_LSB_ADDR = 0X1E;
        const byte BNO055_EULER_P_MSB_ADDR = 0X1F;

        /* Quaternion data registers */
        const byte BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20;
        const byte BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21;
        const byte BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22;
        const byte BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23;
        const byte BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24;
        const byte BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25;
        const byte BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26;
        const byte BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27;

        /* Linear acceleration data registers */
        const byte BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28;
        const byte BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29;
        const byte BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A;
        const byte BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B;
        const byte BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C;
        const byte BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D;

        /* Gravity data registers */
        const byte BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E;
        const byte BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F;
        const byte BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30;
        const byte BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31;
        const byte BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32;
        const byte BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33;

        /* Temperature data register */
        const byte BNO055_TEMP_ADDR = 0X34;

        /* Status registers */
        const byte BNO055_CALIB_STAT_ADDR = 0X35;
        const byte BNO055_SELFTEST_RESULT_ADDR = 0X36;
        const byte BNO055_INTR_STAT_ADDR = 0X37;

        const byte BNO055_SYS_CLK_STAT_ADDR = 0X38;
        const byte BNO055_SYS_STAT_ADDR = 0X39;
        const byte BNO055_SYS_ERR_ADDR = 0X3A;

        /* Unit selection register */
        const byte BNO055_UNIT_SEL_ADDR = 0X3B;
        const byte BNO055_DATA_SELECT_ADDR = 0X3C;

        /* Mode registers */
        const byte BNO055_OPR_MODE_ADDR = 0X3D;
        const byte BNO055_PWR_MODE_ADDR = 0X3E;

        const byte BNO055_SYS_TRIGGER_ADDR = 0X3F;
        const byte BNO055_TEMP_SOURCE_ADDR = 0X40;

        /* Axis remap registers */
        const byte BNO055_AXIS_MAP_CONFIG_ADDR = 0X41;
        const byte BNO055_AXIS_MAP_SIGN_ADDR = 0X42;

        /* SIC registers */
        const byte BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43;
        const byte BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44;
        const byte BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45;
        const byte BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46;
        const byte BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47;
        const byte BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48;
        const byte BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49;
        const byte BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A;
        const byte BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B;
        const byte BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C;
        const byte BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D;
        const byte BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E;
        const byte BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F;
        const byte BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50;
        const byte BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51;
        const byte BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52;
        const byte BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53;
        const byte BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54;

        /* Accelerometer Offset registers */
        const byte ACCEL_OFFSET_X_LSB_ADDR = 0X55;
        const byte ACCEL_OFFSET_X_MSB_ADDR = 0X56;
        const byte ACCEL_OFFSET_Y_LSB_ADDR = 0X57;
        const byte ACCEL_OFFSET_Y_MSB_ADDR = 0X58;
        const byte ACCEL_OFFSET_Z_LSB_ADDR = 0X59;
        const byte ACCEL_OFFSET_Z_MSB_ADDR = 0X5A;

        /* Magnetometer Offset registers */
        const byte MAG_OFFSET_X_LSB_ADDR = 0X5B;
        const byte MAG_OFFSET_X_MSB_ADDR = 0X5C;
        const byte MAG_OFFSET_Y_LSB_ADDR = 0X5D;
        const byte MAG_OFFSET_Y_MSB_ADDR = 0X5E;
        const byte MAG_OFFSET_Z_LSB_ADDR = 0X5F;
        const byte MAG_OFFSET_Z_MSB_ADDR = 0X60;

        /* Gyroscope Offset register s*/
        const byte GYRO_OFFSET_X_LSB_ADDR = 0X61;
        const byte GYRO_OFFSET_X_MSB_ADDR = 0X62;
        const byte GYRO_OFFSET_Y_LSB_ADDR = 0X63;
        const byte GYRO_OFFSET_Y_MSB_ADDR = 0X64;
        const byte GYRO_OFFSET_Z_LSB_ADDR = 0X65;
        const byte GYRO_OFFSET_Z_MSB_ADDR = 0X66;

        /* Radius registers */
        const byte ACCEL_RADIUS_LSB_ADDR = 0X67;
        const byte ACCEL_RADIUS_MSB_ADDR = 0X68;
        const byte MAG_RADIUS_LSB_ADDR = 0X69;
        const byte MAG_RADIUS_MSB_ADDR = 0X6A;
        #endregion

        #region adafruit_bno055_powermode_t

        const byte POWER_MODE_NORMAL = 0X00;
        const byte POWER_MODE_LOWPOWER = 0X01;
        const byte POWER_MODE_SUSPEND = 0X02;

        #endregion

        #region adafruit_bno055_opmode_t

        /* Operation mode settings*/
        const byte OPERATION_MODE_CONFIG = 0X00;
        const byte OPERATION_MODE_ACCONLY = 0X01;
        const byte OPERATION_MODE_MAGONLY = 0X02;
        const byte OPERATION_MODE_GYRONLY = 0X03;
        const byte OPERATION_MODE_ACCMAG = 0X04;
        const byte OPERATION_MODE_ACCGYRO = 0X05;
        const byte OPERATION_MODE_MAGGYRO = 0X06;
        const byte OPERATION_MODE_AMG = 0X07;
        const byte OPERATION_MODE_IMUPLUS = 0X08;
        const byte OPERATION_MODE_COMPASS = 0X09;
        const byte OPERATION_MODE_M4G = 0X0A;
        const byte OPERATION_MODE_NDOF_FMC_OFF = 0X0B;
        const byte OPERATION_MODE_NDOF = 0X0C;

        #endregion

        enum adafruit_bno055_axis_remap_config_t
        {
            REMAP_CONFIG_P0 = 0x21,
            REMAP_CONFIG_P1 = 0x24, // default
            REMAP_CONFIG_P2 = 0x24,
            REMAP_CONFIG_P3 = 0x21,
            REMAP_CONFIG_P4 = 0x24,
            REMAP_CONFIG_P5 = 0x21,
            REMAP_CONFIG_P6 = 0x21,
            REMAP_CONFIG_P7 = 0x24
        }

        enum adafruit_bno055_axis_remap_sign_t
        {
            REMAP_SIGN_P0 = 0x04,
            REMAP_SIGN_P1 = 0x00, // default
            REMAP_SIGN_P2 = 0x06,
            REMAP_SIGN_P3 = 0x02,
            REMAP_SIGN_P4 = 0x03,
            REMAP_SIGN_P5 = 0x01,
            REMAP_SIGN_P6 = 0x07,
            REMAP_SIGN_P7 = 0x05
        }

        #region PROPERTIES

        /// <summary>
        /// Check the accelerometer revision
        /// </summary>
        byte getRevInfo_ACCEL
        {
            get
            {
                return base.Read(BNO055_ACCEL_REV_ID_ADDR);
            }
        }

        /// <summary>
        /// Check the magnetometer revision
        /// </summary>
        byte getRevInfo_MAG
        {
            get
            {
                return base.Read(BNO055_MAG_REV_ID_ADDR);
            }
        }

        /// <summary>
        /// Check the gyroscope revision
        /// </summary>
        byte getRevInfo_GYRO
        {
            get
            {
                return base.Read(BNO055_GYRO_REV_ID_ADDR);
            }
        }

        /// <summary>
        /// Check the BL revision
        /// </summary>
        byte getRevInfo_BL
        {
            get
            {
                return base.Read(BNO055_BL_REV_ID_ADDR);
            }
        }

        /// <summary>
        /// Check the SW revision
        /// </summary>
        UInt16 getRevInfo_SW
        {
            get
            {
                UInt16 swRev = 0;
                byte a = base.Read(BNO055_SW_REV_ID_LSB_ADDR);
                byte b = base.Read(BNO055_SW_REV_ID_MSB_ADDR);
                return (UInt16)((swRev | (b << 8)) | a);
            }
        }

        /// <summary>
        /// Get system calibration state 0 if not calibrated and 3 if fully calibrated
        /// </summary>
        public byte Calibration_SYS
        {
            get
            {
                byte calData = base.Read(BNO055_CALIB_STAT_ADDR);
                return (byte)((calData >> 6) & 0x03);
            }
        }

        /// <summary>
        /// Get gyro calibration state 0 if not calibrated and 3 if fully calibrated
        /// </summary>
        public byte Calibration_GYRO
        {
            get
            {
                byte calData = base.Read(BNO055_CALIB_STAT_ADDR);
                return (byte)((calData >> 4) & 0x03);
            }
        }

        /// <summary>
        /// Get accel calibration state 0 if not calibrated and 3 if fully calibrated
        /// </summary>
        public byte Calibration_ACCEL
        {
            get
            {
                byte calData = base.Read(BNO055_CALIB_STAT_ADDR);
                return (byte)((calData >> 2) & 0x03);
            }
        }

        /// <summary>
        /// Get mag calibration state 0 if not calibrated and 3 if fully calibrated
        /// </summary>
        public byte Calibration_MAG
        {
            get
            {
                byte calData = base.Read(BNO055_CALIB_STAT_ADDR);
                return (byte)(calData & 0x03);
            }
        }

        #endregion

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="address">Optional I2C address</param>
        public BNO055(byte address = BNO055_ADDRESS_A)
        {
            _Configuration = new Microsoft.SPOT.Hardware.I2CDevice.Configuration(address, 400);
        }

        /// <summary>
        /// Check for device is connected, if so initialize.
        /// </summary>
        /// <returns></returns>
        public override bool IsAlive()
        {
            // BNO055 clock stretches for 500us or more!
            //#ifdef ESP8266
            //  Wire.setClockStretchLimit(1000); // Allow for 1000us of clock stretching
            //#endif

            /* Make sure we have the right device */
            byte id = base.Read(BNO055_CHIP_ID_ADDR);
            if (id != BNO055_ID)
            {
                Thread.Sleep(1000); // hold on for boot
                id = base.Read(BNO055_CHIP_ID_ADDR);
                if (id != BNO055_ID)
                {
                    //Debug.Print("BNO055 is not alive.\n\r");
                    return false;  // still not? ok bail
                }
            }

            /* Switch to config mode (just in case since this is the default) */
            setMode(OPERATION_MODE_CONFIG);

            /* Reset */
            base.Write(BNO055_SYS_TRIGGER_ADDR, 0x20);
            while (base.Read(BNO055_CHIP_ID_ADDR) != BNO055_ID)
            {
                Thread.Sleep(10);
            }
            Thread.Sleep(50);

            /* Set to normal power mode */
            base.Write(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
            Thread.Sleep(10);

            base.Write(BNO055_PAGE_ID_ADDR, 0);

            /* Set the output units */
            /*
            uint8_t unitsel = (0 << 7) | // Orientation = Android
                              (0 << 4) | // Temperature = Celsius
                              (0 << 2) | // Euler = Degrees
                              (1 << 1) | // Gyro = Rads
                              (0 << 0);  // Accelerometer = m/s^2
            write8(BNO055_UNIT_SEL_ADDR, unitsel);
            */

            /* Configure axis mapping (see section 3.4) */
            /*
            write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
            delay(10);
            write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
            delay(10);
            */

            base.Write(BNO055_SYS_TRIGGER_ADDR, 0x0);
            Thread.Sleep(10);

            /* Set the requested operating mode (see section 3.3) */
            setMode(OPERATION_MODE_NDOF);
            Thread.Sleep(20);

            return true;
        }

        public override bool Read()
        {
            //DECIDE WHAT DATA TO HAVE THIS METHOD OUTPUT --- I DID THIS TO YOU

            return true;
        }

        /// <summary>
        /// Puts the chip in the specified operating mode
        /// </summary>
        /// <param name="mode"></param>
        void setMode(byte mode)
        {
            adafruit_bno055_opmode_t_mode = mode;
            base.Write(BNO055_OPR_MODE_ADDR, adafruit_bno055_opmode_t_mode);
            Thread.Sleep(30);
        }

        /// <summary>
        /// Check calibration
        /// </summary>
        /// <returns></returns>
        public bool isFullyCalibrated()
        {
            if (Calibration_SYS < 3 || Calibration_GYRO < 3 || Calibration_ACCEL < 3 || Calibration_MAG < 3)
                return false;
            return true;
        }

        /// <summary>
        /// Use the external 32.768KHz crystal
        /// </summary>
        /// <param name="usextal"></param>
        void setExtCrystalUse(bool usextal)
        {
            byte modeback = adafruit_bno055_opmode_t_mode;

            /* Switch to config mode (just in case since this is the default) */
            setMode(OPERATION_MODE_CONFIG);

            Thread.Sleep(25);

            base.Write(BNO055_PAGE_ID_ADDR, 0);
            if (usextal)
            {
                base.Write(BNO055_SYS_TRIGGER_ADDR, 0x80);
            }
            else
            {
                base.Write(BNO055_SYS_TRIGGER_ADDR, 0x00);
            }

            Thread.Sleep(10);

            /* Set the requested operating mode (see section 3.3) */
            setMode(modeback);

            Thread.Sleep(20);
        }

        /// <summary>
        /// Gets the latest system status info
        /// </summary>
        /// <param name="system_status"></param>
        /// <param name="self_test_result"></param>
        /// <param name="system_error"></param>
        public void getSystemStatus(ref byte system_status, ref byte self_test_result, ref byte system_error)
        {
            base.Write(BNO055_PAGE_ID_ADDR, 0);

            /* System Status (see section 4.3.58)
               ---------------------------------
               0 = Idle
               1 = System Error
               2 = Initializing Peripherals
               3 = System Iniitalization
               4 = Executing Self-Test
               5 = Sensor fusio algorithm running
               6 = System running without fusion algorithms */

            //if (system_status != 0)
            system_status = base.Read(BNO055_SYS_STAT_ADDR);

            /* Self Test Results (see section )
               --------------------------------
               1 = test passed, 0 = test failed

               Bit 0 = Accelerometer self test
               Bit 1 = Magnetometer self test
               Bit 2 = Gyroscope self test
               Bit 3 = MCU self test

               0x0F = all good! */

            //if (self_test_result != 0)
            self_test_result = base.Read(BNO055_SELFTEST_RESULT_ADDR);

            /* System Error (see section 4.3.59)
               ---------------------------------
               0 = No error
               1 = Peripheral initialization error
               2 = System initialization error
               3 = Self test result failed
               4 = Register map value out of range
               5 = Register map address out of range
               6 = Register map write error
               7 = BNO low power mode not available for selected operat ion mode
               8 = Accelerometer power mode not available
               9 = Fusion algorithm configuration error
               A = Sensor configuration error */

            //if (system_error != 0)
            system_error = base.Read(BNO055_SYS_ERR_ADDR);

            Thread.Sleep(200);
        }

        /// <summary>
        /// Gets the temperature in degrees celsius
        /// </summary>
        /// <returns></returns>
        public int getTemperature()
        {
            return (int)(base.Read(BNO055_TEMP_ADDR));
        }

        Vector VectorOrientation = new Vector();

        /// <summary>
        /// Gets a vector reading from the specified source
        /// </summary>
        /// <param name="vector_type"></param>
        /// <returns></returns>
        public Vector getVector(adafruit_vector_type_t vector_type)
        {
            byte[] buffer = new byte[6];

            Int16 x, y, z;
            x = y = z = 0;

            /* Read vector data (6 bytes) */
            buffer = base.Read((byte)vector_type, 6);

            x = (Int16)((buffer[0]) | ((buffer[1]) << 8));
            y = (Int16)((buffer[2]) | ((buffer[3]) << 8));
            z = (Int16)((buffer[4]) | ((buffer[5]) << 8));

            /* Convert the value to an appropriate range (section 3.6.4) */
            /* and assign the value to the Vector type */
            switch (vector_type)
            {
                case adafruit_vector_type_t.VECTOR_MAGNETOMETER:
                    /* 1uT = 16 LSB */
                    VectorOrientation.x = ((double)x) / 16.0;
                    VectorOrientation.y = ((double)y) / 16.0;
                    VectorOrientation.z = ((double)z) / 16.0;
                    break;
                case adafruit_vector_type_t.VECTOR_GYROSCOPE:
                    /* 1rps = 900 LSB */
                    VectorOrientation.x = ((double)x) / 900.0;
                    VectorOrientation.y = ((double)y) / 900.0;
                    VectorOrientation.z = ((double)z) / 900.0;
                    break;
                case adafruit_vector_type_t.VECTOR_EULER:
                    /* 1 degree = 16 LSB */
                    VectorOrientation.x = ((double)x) / 16.0;
                    VectorOrientation.y = ((double)y) / 16.0;
                    VectorOrientation.z = ((double)z) / 16.0;
                    break;
                case adafruit_vector_type_t.VECTOR_ACCELEROMETER:
                case adafruit_vector_type_t.VECTOR_LINEARACCEL:
                case adafruit_vector_type_t.VECTOR_GRAVITY:
                    /* 1m/s^2 = 100 LSB */
                    VectorOrientation.x = ((double)x) / 100.0;
                    VectorOrientation.y = ((double)y) / 100.0;
                    VectorOrientation.z = ((double)z) / 100.0;
                    break;
            }

            return VectorOrientation;
        }

        /// <summary>
        /// Updates sensor offsets.
        /// </summary>
        /// <param name="BNO_SENSOR_OFFSETS"></param>
        /// <returns></returns>
        public bool getSensorOffsets()
        {
            if (isFullyCalibrated())
            {
                byte lastMode = adafruit_bno055_opmode_t_mode;
                setMode(OPERATION_MODE_CONFIG);
                Thread.Sleep(25);

                BNO_SENSOR_OFFSETS.accel_offset_x = (UInt16)((base.Read(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (base.Read(ACCEL_OFFSET_X_LSB_ADDR)));
                BNO_SENSOR_OFFSETS.accel_offset_y = (UInt16)((base.Read(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (base.Read(ACCEL_OFFSET_Y_LSB_ADDR)));
                BNO_SENSOR_OFFSETS.accel_offset_z = (UInt16)((base.Read(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (base.Read(ACCEL_OFFSET_Z_LSB_ADDR)));

                BNO_SENSOR_OFFSETS.gyro_offset_x = (UInt16)((base.Read(GYRO_OFFSET_X_MSB_ADDR) << 8) | (base.Read(GYRO_OFFSET_X_LSB_ADDR)));
                BNO_SENSOR_OFFSETS.gyro_offset_y = (UInt16)((base.Read(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (base.Read(GYRO_OFFSET_Y_LSB_ADDR)));
                BNO_SENSOR_OFFSETS.gyro_offset_z = (UInt16)((base.Read(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (base.Read(GYRO_OFFSET_Z_LSB_ADDR)));

                BNO_SENSOR_OFFSETS.mag_offset_x = (UInt16)((base.Read(MAG_OFFSET_X_MSB_ADDR) << 8) | (base.Read(MAG_OFFSET_X_LSB_ADDR)));
                BNO_SENSOR_OFFSETS.mag_offset_y = (UInt16)((base.Read(MAG_OFFSET_Y_MSB_ADDR) << 8) | (base.Read(MAG_OFFSET_Y_LSB_ADDR)));
                BNO_SENSOR_OFFSETS.mag_offset_z = (UInt16)((base.Read(MAG_OFFSET_Z_MSB_ADDR) << 8) | (base.Read(MAG_OFFSET_Z_LSB_ADDR)));

                BNO_SENSOR_OFFSETS.accel_radius = (UInt16)((base.Read(ACCEL_RADIUS_MSB_ADDR) << 8) | (base.Read(ACCEL_RADIUS_LSB_ADDR)));
                BNO_SENSOR_OFFSETS.mag_radius = (UInt16)((base.Read(MAG_RADIUS_MSB_ADDR) << 8) | (base.Read(MAG_RADIUS_LSB_ADDR)));

                setMode(lastMode);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Writes to the sensor's offset registers from an offset
        /// </summary>
        /// <param name="offsets_type"></param>
        public void setSensorOffsets(adafruit_bno055_offsets_t offsets_type)
        {
            byte lastMode = adafruit_bno055_opmode_t_mode;

            setMode(OPERATION_MODE_CONFIG);

            Thread.Sleep(25);

            base.Write(ACCEL_OFFSET_X_LSB_ADDR, (byte)((offsets_type.accel_offset_x) & 0x0FF));
            base.Write(ACCEL_OFFSET_X_MSB_ADDR, (byte)((offsets_type.accel_offset_x >> 8) & 0x0FF));
            base.Write(ACCEL_OFFSET_Y_LSB_ADDR, (byte)((offsets_type.accel_offset_y) & 0x0FF));
            base.Write(ACCEL_OFFSET_Y_MSB_ADDR, (byte)((offsets_type.accel_offset_y >> 8) & 0x0FF));
            base.Write(ACCEL_OFFSET_Z_LSB_ADDR, (byte)((offsets_type.accel_offset_z) & 0x0FF));
            base.Write(ACCEL_OFFSET_Z_MSB_ADDR, (byte)((offsets_type.accel_offset_z >> 8) & 0x0FF));

            base.Write(GYRO_OFFSET_X_LSB_ADDR, (byte)((offsets_type.gyro_offset_x) & 0x0FF));
            base.Write(GYRO_OFFSET_X_MSB_ADDR, (byte)((offsets_type.gyro_offset_x >> 8) & 0x0FF));
            base.Write(GYRO_OFFSET_Y_LSB_ADDR, (byte)((offsets_type.gyro_offset_y) & 0x0FF));
            base.Write(GYRO_OFFSET_Y_MSB_ADDR, (byte)((offsets_type.gyro_offset_y >> 8) & 0x0FF));
            base.Write(GYRO_OFFSET_Z_LSB_ADDR, (byte)((offsets_type.gyro_offset_z) & 0x0FF));
            base.Write(GYRO_OFFSET_Z_MSB_ADDR, (byte)((offsets_type.gyro_offset_z >> 8) & 0x0FF));

            base.Write(MAG_OFFSET_X_LSB_ADDR, (byte)((offsets_type.mag_offset_x) & 0x0FF));
            base.Write(MAG_OFFSET_X_MSB_ADDR, (byte)((offsets_type.mag_offset_x >> 8) & 0x0FF));
            base.Write(MAG_OFFSET_Y_LSB_ADDR, (byte)((offsets_type.mag_offset_y) & 0x0FF));
            base.Write(MAG_OFFSET_Y_MSB_ADDR, (byte)((offsets_type.mag_offset_y >> 8) & 0x0FF));
            base.Write(MAG_OFFSET_Z_LSB_ADDR, (byte)((offsets_type.mag_offset_z) & 0x0FF));
            base.Write(MAG_OFFSET_Z_MSB_ADDR, (byte)((offsets_type.mag_offset_z >> 8) & 0x0FF));

            base.Write(ACCEL_RADIUS_LSB_ADDR, (byte)((offsets_type.accel_radius) & 0x0FF));
            base.Write(ACCEL_RADIUS_MSB_ADDR, (byte)((offsets_type.accel_radius >> 8) & 0x0FF));

            base.Write(MAG_RADIUS_LSB_ADDR, (byte)((offsets_type.mag_radius) & 0x0FF));
            base.Write(MAG_RADIUS_MSB_ADDR, (byte)((offsets_type.mag_radius >> 8) & 0x0FF));

            setMode(lastMode);

            Thread.Sleep(25);
        }


        //public Quaternion getQuat()
        //{
        //  byte[] buffer = new byte[8];

        //  UInt16 x, y, z, w;
        //  x = y = z = w = 0;

        //  /* Read quat data (8 bytes) */
        //  buffer = base.Read(BNO055_QUATERNION_DATA_W_LSB_ADDR,  8);
        //  w = (UInt16)(((buffer[1]) << 8) | buffer[0]);
        //  x = (UInt16)(((buffer[3]) << 8) | buffer[2]);
        //  y = (UInt16)(((buffer[5]) << 8) | buffer[4]);
        //  z = (UInt16)(((buffer[7]) << 8) | buffer[6]);

        //  /* Assign to Quaternion */
        //  /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
        //     3.6.5.5 Orientation (Quaternion)  */
        //  const double scale = (1.0 / (1<<14));
        //  Quaternion quat(scale * w, scale * x, scale * y, scale * z);
        //  return quat;
        //}
    }
}
