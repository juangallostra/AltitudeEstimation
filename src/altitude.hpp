/*
    altitude.hpp: Altitude estimation via barometer/accelerometer fusion
*/

# pragma once

#include "datatypes.hpp"
#include "sensors/barometer.hpp"
#include "sensors/imu.hpp"
#include "filters/filters.hpp"
#include "algebra/algebra.hpp"

class AltitudeEstimator {
  private:
    // sensor abstractions
    Barometer baro = Barometer();
    IMU imu = IMU();
    // required parameters for the filters used for the estimations
    // sensor's standard deviations
    float sigmaAccel = 0.2;
    float sigmaGyro = 0.2;
    float sigmaBaro = 5;
    // gravity
    float g = 9.81;
    // Acceleration markov chain model state transition constant
    float ca = 0.5;
    // Zero-velocity update acceleration threshold
    float accelThreshold = 0.3;
    // Sampling period
    float previousTime = millis();
    // required filters for altitude and vertical velocity estimation
    KalmanFilter kalman = KalmanFilter(ca, sigmaGyro, sigmaAccel);
    ComplementaryFilter complementary = ComplementaryFilter(sigmaAccel, sigmaBaro, accelThreshold);
    // Estimated past vertical acceleration
    float pastVerticalAccel = 0;
    float pastVerticalVelocity = 0;
    float pastAltitude = 0;
    float pastGyro[3] = {0, 0, 0};
    float pastAccel[3] = {0, 0, 0};
    // fake calibration
    int count = 0;

  public:
    // estimated altitude and vertical velocity
    float estimatedAltitude = 0;
    float estimatedVelocity = 0;

    void init(void)
    {
        baro.init();
    }

    void updateBaro(bool armed, float pressure)
    {
        baro.update(pressure);
        // Calibrate barometer when the drone is resting
        if (!armed && count < 100){
          baro.calibrate();
          count++;
          return;
        }
        return;
    }

    void updateAcceleration(float _accel[3])
    {
        imu.updateAcceleration(_accel);
    }

    void updateGyro(float _gyro[3])
    {
        imu.updateGyro(_gyro);
    }

    float estimate()
    {
        float currentTime = millis();
        float verticalAccel = kalman.estimate(pastGyro,
                                              pastAccel,
                                              (currentTime-previousTime)/1000.0);
        complementary.estimate(& estimatedVelocity,
                               & estimatedAltitude,
                               baro.getAltitude(),
                               pastAltitude,
                               pastVerticalVelocity,
                               pastVerticalAccel,
                               (currentTime-previousTime)/1000.0);
        // update values for next iteration
        copyVector(pastGyro, imu.gyro);
        copyVector(pastAccel, imu.accel);
        pastAltitude = estimatedAltitude;
        pastVerticalVelocity = estimatedVelocity;
        pastVerticalAccel = verticalAccel;
        previousTime = currentTime;
        return estimatedAltitude;
    }

    float getAltitude()
    {
      // return the last estimated altitude
      return pastAltitude;
    }

}; // class AltitudeEstimator
