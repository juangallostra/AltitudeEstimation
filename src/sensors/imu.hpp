/*
   imu.hpp: Altitude estimation via accelerometer Z-axis integration
*/

#pragma once

#include <math.h>

class IMU {

    private:

        // This variables will store the latest available readings
        // from the gyro and the IMU. Since this class is Hardware
        // independent this values will have to be supplied

    public:

      float gyro[3];
      float accel[3];
      // Update last known acceleration values and time of the last measure
      void updateAcceleration(float _accel[3])
      {
          memcpy(accel, _accel, 3*sizeof(float));
      }

      // Update last known gyro values and time of the last measure
      void updateGyro(float _gyro[3])
      {
          memcpy(gyro, _gyro, 3*sizeof(float));
      }

}; // class IMU
