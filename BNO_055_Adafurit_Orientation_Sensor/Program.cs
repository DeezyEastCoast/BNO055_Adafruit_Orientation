using System;
using System.Collections;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Presentation;
using Microsoft.SPOT.Presentation.Controls;
using Microsoft.SPOT.Presentation.Media;
using Microsoft.SPOT.Presentation.Shapes;
using Microsoft.SPOT.Touch;

using Gadgeteer.Networking;
using GT = Gadgeteer;
using GTM = Gadgeteer.Modules;

namespace BNO_055_Adafurit_Orientation_Sensor
{
    public partial class Program
    {

        #region Adafruit Orientation Sensor

        static BNO055 IMU_BNO = new BNO055();
        byte[] sysStatus = new byte[] { 0, 0, 0 };
        static BNO055.Vector myVector;

        #endregion
   
        void ProgramStarted()
        {
            // Use Debug.Print to show messages in Visual Studio's "Output" window during debugging.
            Debug.Print("Program Started");

            bool IMU_BNO_ALIVE = IMU_BNO.IsAlive();

            while (true)
            {
                myVector = IMU_BNO.getVector(BNO055.adafruit_vector_type_t.VECTOR_EULER);
                Debug.Print("Euler: X:" + myVector.x + " Y:" + myVector.y + " Z:" + myVector.z + "\n\r"); //out put orientation, heading, pitch, roll angles.

                //output calibration levels see Adafruit "Learn" webpage about how to calibrate BNO055
                Debug.Print("SYS Cal: " + IMU_BNO.Calibration_SYS + "\n\r");
                Debug.Print("Mag Cal: " + IMU_BNO.Calibration_MAG + "\n\r");
                Debug.Print("Accel Cal: " + IMU_BNO.Calibration_ACCEL + "\n\r");
                Debug.Print("Gyro Cal: " + IMU_BNO.Calibration_GYRO + "\n\r");

                //output system status
                IMU_BNO.getSystemStatus(ref sysStatus[0], ref sysStatus[1], ref sysStatus[2]);
                Debug.Print("SystemStatus: " + sysStatus[0].ToString() + " -- " + sysStatus[1].ToString() + " -- " + sysStatus[2].ToString() + "\n\r");

                Thread.Sleep(1000);
            }

        }
    }
}
