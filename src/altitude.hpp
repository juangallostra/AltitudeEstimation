/*
    altitude.hpp: Altitude estimation via barometer/accelerometer fusion
*/

# pragma once

#include "datatypes.hpp"
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
    uint32_t previousTime = micros();
    // required filters for altitude and vertical velocity estimation
    KalmanFilter kalman;
    ComplementaryFilter complementary;
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
    :kalman(ca, sigmaGyro, sigmaAccel), complementary(sigmaAccel, sigmaBaro, accelThreshold)
    {
      this->sigmaAccel = sigmaAccel;
      this->sigmaGyro = sigmaGyro;
      this->sigmaBaro = sigmaBaro;
      this->ca = ca;
      this->accelThreshold = accelThreshold;

    }

    void estimate(float accel[3], float gyro[3], float baroHeight, uint32_t timestamp)
    {
        float deltat = (float)(timestamp-previousTime)/1000000.0f;
        float verticalAccel = kalman.estimate(pastGyro,
                                              pastAccel,
                                              deltat);
        complementary.estimate(& estimatedVelocity,
                               & estimatedAltitude,
                               baroHeight,
                               pastAltitude,
                               pastVerticalVelocity,
                               pastVerticalAccel,
                               deltat);
        // update values for next iteration
        copyVector(pastGyro, gyro);
        copyVector(pastAccel, accel);
        pastAltitude = estimatedAltitude;
        pastVerticalVelocity = estimatedVelocity;
        pastVerticalAccel = verticalAccel;
        previousTime = timestamp;
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

    float getVerticalAcceleration()
    {
        // return the last estimated vertical acceleration
        return pastVerticalAccel;
    }

}; // class AltitudeEstimator
