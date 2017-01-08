using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace BNO_055_Adafurit_Orientation_Sensor
{
    public abstract class SensorBase
    {
        /// <summary>
        /// The RawX value of the sensor
        /// </summary>
        public float RawX { get; protected set; }

        /// <summary>
        /// The RawY value of the sensor
        /// </summary>
        public float RawY { get; protected set; }

        /// <summary>
        /// The RawZ value of the sensor
        /// </summary>
        public float RawZ { get; protected set; }

        /// <summary>
        /// The two's complement decoded X value, if supported by sensor.
        /// </summary>
        public float X { get; protected set; }

        /// <summary>
        /// The two's complement decoded Y value, if supported by sensor.
        /// </summary>
        public float Y { get; protected set; }

        /// <summary>
        /// The two's complement decoded Z value, if supported by sensor.
        /// </summary>
        public float Z { get; protected set; }

        public float Temperature { get; set; }
        public float Pressure { get; set; }

        protected I2CDevice.Configuration _Configuration;
        private static I2CDevice _I2CDevice;

        public SensorBase()
        {
            if (_I2CDevice == null)
                _I2CDevice = new I2CDevice(new I2CDevice.Configuration(0x00, 400));

        }

        /// <summary>
        /// Class implementation to check if the I2CDevice is responsive.
        /// </summary>
        /// <returns></returns>
        public abstract bool IsAlive();

        /// <summary>
        /// Class implementation to read from the I2CDevice.
        /// </summary>
        /// <returns>A boolean value that represent the read is completed</returns>
        public abstract bool Read();

        /// <summary>
        /// This method reads a single a single byte from the particular registry.
        /// </summary>
        /// <param name="reg">Register to read</param>
        /// <returns>The value read from the registry</returns>
        public byte Read(byte reg)
        {
            _I2CDevice.Config = _Configuration;
            var buffer = new byte[1];
            I2CDevice.I2CTransaction[] transaction;
            transaction = new I2CDevice.I2CTransaction[]
             {
                 I2CDevice.CreateWriteTransaction(new byte[] { reg }),
                 I2CDevice.CreateReadTransaction(buffer)
             };

            int result = _I2CDevice.Execute(transaction, 1000);
            return buffer[0];
        }

        /// <summary>
        /// If device supports multiple reads. Use this method to read the device starting from the supplied registry and the length of the output buffer.
        /// </summary>
        /// <param name="reg">The device registry</param>
        /// <param name="bufferLength">Length for the output buffered</param>
        /// <returns>The output of the read byte(s)</returns>
        public byte[] Read(byte reg, int bufferLength)
        {
            _I2CDevice.Config = _Configuration;
            var buffer = new byte[bufferLength];
            I2CDevice.I2CTransaction[] transactions = new I2CDevice.I2CTransaction[]
                 {
                     I2CDevice.CreateWriteTransaction(new byte[] { reg }),
                     I2CDevice.CreateReadTransaction(buffer)
                 };

            int result = _I2CDevice.Execute(transactions, 1000);
            return buffer;

        }

        /// <summary>
        /// Writes a byte value to the specific I2CDevice registry
        /// </summary>
        /// <param name="reg">The registry to write to.</param>
        /// <param name="value">The registry to read from.</param>
        /// <returns>The write operation was completed successfully</returns>
        public bool Write(byte reg, byte value)
        {
            _I2CDevice.Config = _Configuration;
            return _I2CDevice.Execute(new I2CDevice.I2CTransaction[] { I2CDevice.CreateWriteTransaction(new byte[] { reg, value }) }, 1000) > 0;
        }


        /// <summary>
        /// Calculate the two's complement from a 16 bit value
        /// </summary>
        /// <param name="lsb">The least significant bit </param>
        /// <param name="msb"></param>
        /// <returns>The short value of the calculat</returns>
        protected Int16 CalculateTwoComplement16(byte lsb, byte msb)
        {

            var val = (Int16)(lsb | (msb << 8));
            var x = (short)((((val >> 15) == 1) ? -65536 : 0) + (val & 0xFFFF));
            return x;
        }



    }
}
