/*
   AltitudeEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>
#include <Wire.h>
// Assuming the IMU is an MPU9250 and thr baro a MS5637
#include <MPU9250_Passthru.h>
#include <MS5637.h>

#include "altitude.h"

// helper variables and functions for obtaining baro data
static const uint8_t HISTORY_SIZE = 48;

static float   groundAltitude = 0;
static float   groundPressure = 0;
static float   pressureSum = 0;
static float   history[HISTORY_SIZE];
static uint8_t historyIdx = 0;
static int     endCalibration = 120;

static MS5637 barometer = MS5637();

// Pressure in millibars to altitude in meters. We assume
// millibars are the units of the pressure readings from the sensor
static float millibarsToMeters(float mbar)
{
    // see: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
    return (1.0f - powf(mbar / 1013.25f, 0.190295f)) * 44330.0f;
}

// Calibrate the baro by setting the average measured pressure as the
// ground pressure and the corresponding altitude as the ground altitude.
static void calibrate(float pressure)
{
    // Update pressure history
    history[historyIdx] = pressure;
    pressureSum += pressure;
    // cycle the index throught the history array
    uint8_t nextIndex = (historyIdx + 1) % HISTORY_SIZE;
    // Remove next reading from sum so that pressureSum is kept in sync
    pressureSum -= history[nextIndex];
    historyIdx = nextIndex;
    // groundPressure will stabilize at 8 times the average measured
    // pressure (when groundPressure/8 equals pressureSum/(HISTORY_SIZE-1))
    // This acts as a low pass filter and helps to reduce noise
    groundPressure -= groundPressure / 8;
    groundPressure += pressureSum / (HISTORY_SIZE - 1);
    groundAltitude = millibarsToMeters(groundPressure/8);
}

static float getAltitude(float pressure)
{
    return  (millibarsToMeters(pressure) - groundAltitude);
}

// helper variables and functions for obtaining IMU data
// Sensor full-scale settings
static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_2G;
static const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_2000DPS;
static const MPU9250::Mscale_t MSCALE = MPU9250::MFS_16BITS;
static const MPU9250::Mmode_t MMODE = MPU9250::M_100Hz;
// SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
// SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
static const uint8_t SAMPLE_RATE_DIVISOR = 0;
// MPU9250 add-on board has interrupt on Butterfly pin 8
static const uint8_t INTERRUPT_PIN = 8;

// Use the MPU9250 in pass-through mode
static MPU9250_Passthru imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);
// flag for when new data is received
static bool gotNewData = false;

static void interruptHandler()
{
    gotNewData = true;
}

static void getGyrometerAndAccelerometer(float gyro[3], float accel[3])
{
    if (gotNewData) {

        gotNewData = false;

        if (imu.checkNewAccelGyroData()) {
            float ax, ay, az, gx, gy, gz;
            imu.readAccelerometer(ay, ax, az);
            imu.readGyrometer(gy, gx, gz);
            gx = -gx;
            // Copy gyro values back out in rad/sec
            gyro[0] = gx * M_PI / 180.0f;
            gyro[1] = gy * M_PI / 180.0f;
            gyro[2] = gz * M_PI / 180.0f;
            // and acceleration values
            accel[0] = ax;
            accel[1] = ay;
            accel[2] = az;
        } // if (imu.checkNewAccelGyroData())

    } // if gotNewData

}

// Altitude estimator
static AltitudeEstimator altitude = AltitudeEstimator(0.0005, // sigma Accel
        0.0005, // sigma Gyro
        0.018,   // sigma Baro
        0.5, // ca
        0.1);// accelThreshold

void setup(void)
{
    // Start I^2C
    Wire.begin();
    Wire.setClock(400000); // I2C frequency at 400 kHz
    delay(1000);

    // initialize the MPU9250
    imu.begin();
    
    // Set all pressure history entries to 0
    for (uint8_t k = 0; k < HISTORY_SIZE; ++k) {
        history[k] = 0;
    }
    // begin Barometer
    barometer.begin();

    // calibrate barometer
    for (uint8_t k=0; k <= endCalibration; k++) {
        float readPressure;
        barometer.getPressure(& readPressure);
        calibrate(readPressure);
    }
    // Begin serial comms
    Serial.begin(115200);
    // Set up the interrupt pin, it's set as active high, push-pull
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);

}

void loop(void)
{
    // get all necessary data
    float pressure;
    barometer.getPressure(& pressure);
    float baroHeight = getAltitude(pressure);
    uint32_t timestamp = micros();
    float accelData[3];
    float gyroData[3];
    getGyrometerAndAccelerometer(gyroData, accelData);
    altitude.estimate(accelData, gyroData, baroHeight, timestamp);
    Serial.print(baroHeight);
    Serial.print(",");
    Serial.print(altitude.getAltitude());
    Serial.print(",");
    Serial.print(altitude.getVerticalVelocity());
    Serial.print(",");
    Serial.println(altitude.getVerticalAcceleration());
}
