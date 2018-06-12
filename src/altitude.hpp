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
    // required parameters for the filters used for the estimations
    // sensor's standard deviations
    float sigmaAccel;
    float sigmaGyro;
    float sigmaBaro;
    // Acceleration markov chain model state transition constant
    float ca;
    // Zero-velocity update acceleration threshold
    float accelThreshold;
    // gravity
    float g = 9.81;
    // For computing the sampling period
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
    // estimated altitude and vertical velocity
    float estimatedAltitude = 0;
    float estimatedVelocity = 0;

  public:

    AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro,
                           float ca, float accelThreshold)
    {
      this->sigmaAccel = sigmaAccel;
      this->sigmaGyro = sigmaGyro;
      this->sigmaBaro = sigmaBaro;
      this->ca = ca;
      this->accelThreshold = accelThreshold;
    }

    void estimate(float accel[3], float gyro[3], float height)
    {
        float currentTime = millis();
        float verticalAccel = kalman.estimate(pastGyro,
                                              pastAccel,
                                              (currentTime-previousTime)/1000.0);
        complementary.estimate(& estimatedVelocity,
                               & estimatedAltitude,
                               height,
                               pastAltitude,
                               pastVerticalVelocity,
                               pastVerticalAccel,
                               (currentTime-previousTime)/1000.0);
        // update values for next iteration
        copyVector(pastGyro, gyro);
        copyVector(pastAccel, accel);
        pastAltitude = estimatedAltitude;
        pastVerticalVelocity = estimatedVelocity;
        pastVerticalAccel = verticalAccel;
        previousTime = currentTime;
    }

    float getAltitude()
    {
      // return the last estimated altitude
      return estimatedAltitude;
    }

    float getVerticalVelocity()
    {
      // return the last estimated vertical velocity
      return estimatedVelocity;
    }

}; // class AltitudeEstimator
