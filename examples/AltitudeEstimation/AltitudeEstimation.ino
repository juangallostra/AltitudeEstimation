/*
   AltitudeEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>
#include <Wire.h>
// IMU
#include <LSM6DSM.h>
// Barometer
#include <LPS22HB.h>
#include <VL53L1X.h>
// For quaternion computation
#include <quaternion.hpp>

#include "altitude.h"

uint8_t LED_PIN = 38;

// Global constants for 6 DoF quaternion filter
const float GYRO_MEAS_ERROR = M_PI * (40.0f / 180.0f);
const float GYRO_MEAS_DRIFT = M_PI * (0.0f  / 180.0f);
const float BETA = sqrtf(3.0f / 4.0f) * GYRO_MEAS_ERROR; 
const float ZETA = sqrt(3.0f / 4.0f) * GYRO_MEAS_DRIFT;

// Quaternion support: even though IMU has a magnetometer, we keep it simple for now by 
// using a 6DOF fiter (accel, gyro)
quat::MadgwickQuaternionFilter6DOF _quaternionFilter = quat::MadgwickQuaternionFilter6DOF(BETA, ZETA);

void getQuaternion(float quat[4], float accel[3], float gyro[3], float deltat)
{
    // Run the quaternion on the IMU values acquired in imuRead()                   
    _quaternionFilter.update(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], deltat); 

    // Copy the quaternion back out
    quat[0] = _quaternionFilter.q1;
    quat[1] = _quaternionFilter.q2;
    quat[2] = _quaternionFilter.q3;
    quat[3] = _quaternionFilter.q4;
}

// --- IMU related variables and functions ---
// LSM6DSM settings
static const LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_2G;
static const LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_2000DPS;
static const LSM6DSM::Rate_t   AODR   = LSM6DSM::ODR_1660Hz;
static const LSM6DSM::Rate_t   GODR   = LSM6DSM::ODR_1660Hz;

// Biases
float ACCEL_BIAS[3] = {0.0, 0.0, 0.0};
float GYRO_BIAS[3]  = {0.0, 0.0, 0.0};

LSM6DSM lsm6dsm = LSM6DSM(Ascale, Gscale, AODR, GODR, ACCEL_BIAS, GYRO_BIAS);

static void getGyrometerAndAccelerometer(float gyro[3], float accel[3], float _q[4], float deltat)
{
    float _ax, _ay, _az, _gx, _gy, _gz;
    
    if (lsm6dsm.checkNewData()) {
        lsm6dsm.readData(_ax, _ay, _az, _gx, _gy, _gz);

        // Negate to support board orientation
        _ax = -_ax;
        _gy = -_gy;
        _gz = -_gz;

        // Copy gyro values back out in rad/sec
        gyro[0] = _gx * M_PI / 180.0f;
        gyro[1] = _gy * M_PI / 180.0f;
        gyro[2] = _gz * M_PI / 180.0f;
        // and acceleration values
        accel[0] = _ax;
        accel[1] = _ay;
        accel[2] = _az;

    } 
    // Update quaternion estimation
    getQuaternion(_q, accel, gyro, deltat);
}

static VL53L1X distanceSensor;

// Helper function to compute euler angles
static void computeEulerAngles(float q[4], float euler[3])
{
    euler[0] = atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
    euler[1] =  asin(2.0f*(q[1]*q[3]-q[0]*q[2]));
    euler[2] = atan2(2.0f*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);
}


// Altitude estimator
static AltitudeEstimator altitude = AltitudeEstimator(0.0005, // sigma Accel
        0.0005,   // sigma Gyro
        0.0001,    // sigma Range
        0.5,      // ca
        0.1);     // accelThreshold

// We will control when the data is sent through serial
float printTime = millis();
float currentTime = millis();

void setup(void)
{
    // Start I^2C
    Wire.begin();
    Wire.setClock(400000); // I2C frequency at 400 kHz
    delay(1000);

    // Begin serial comms
    Serial.begin(115200);
    // initialize sensors
    lsm6dsm.begin();
    lsm6dsm.calibrate(GYRO_BIAS, ACCEL_BIAS);
    if (distanceSensor.begin() == false) {
        while (true) {
            Serial.println("Sensor offline!");
            delay(200);
        }
    }
}

void loop(void)
{
  // print data every 20 millis
  // (Lidar update frequency is 50Hz)
  currentTime = millis();
  float deltat = currentTime - printTime;
  if (deltat > 20)
  {
    // get all necessary data
    float accelData[3];
    float gyroData[3];
    float quaternion[4];
    getGyrometerAndAccelerometer(gyroData, accelData, quaternion, deltat / 1000.0f);
    float rangeHeight = (float)distanceSensor.getDistance() / 1000.0f;
    uint32_t timestamp = micros();
    // Compensate for effect of pitch, roll on rangefinder reading
    float euler[3];
    computeEulerAngles(quaternion, euler);
    rangeHeight =  rangeHeight * cos(euler[0]) * cos(euler[1]);
    altitude.estimate(accelData, gyroData, rangeHeight, timestamp);
    Serial.print(rangeHeight);
    Serial.print(",");
    Serial.print(altitude.getAltitude());
    Serial.print(",");
    Serial.print(altitude.getVerticalVelocity());
    Serial.print(",");
    Serial.println(altitude.getVerticalAcceleration());
    printTime = currentTime;
  }
}
