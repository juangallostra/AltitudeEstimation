/*
   SENtralAltitudeEstimation.ino : Arduino sketch to perform altitude estimation using
   the EM7180 SENtral Sensor Fusion Solution.
 */

#include <Arduino.h>
#include <Wire.h>

#include <EM7180.h> 

#include "altitude.hpp"

// helper variables and functions for obtaining baro data
static const uint8_t HISTORY_SIZE = 48;

static float   groundAltitude = 0;
static float   groundPressure = 0;
static float   pressureSum = 0;
static float   history[HISTORY_SIZE];
static uint8_t historyIdx = 0;
static int     endCalibration = 120;

static const uint8_t  ARES           = 8;    // Gs
static const uint16_t GRES           = 2000; // degrees per second
static const uint16_t MRES           = 1000; // microTeslas
const uint8_t         MAG_RATE       = 100;  // Hz
static const uint16_t ACCEL_RATE     = 330;  // Hz
static const uint16_t GYRO_RATE      = 330;  // Hz
static const uint8_t  BARO_RATE      = 50;   // Hz
static const uint8_t  Q_RATE_DIVISOR = 5;    // 1/5 gyro rate

static EM7180_Master sentral = EM7180_Master(ARES, GRES, MRES, MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

static float gyroAdcToRadians;

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

/*
static bool gotNewData;
static void interruptHandler()
{
    gotNewData = true;
}*/

// Altitude estimator
static AltitudeEstimator altitude = AltitudeEstimator(0.0005, // sigma Accel
        0.0005, // sigma Gyro
        0.018,   // sigma Baro
        0.5, // ca
        0.1);// accelThreshold

void setup(void)
{
    // Set all pressure history entries to 0
    for (uint8_t k = 0; k < HISTORY_SIZE; ++k) {
        history[k] = 0;
    }
    // begin Barometer

    // calibrate barometer
    for (uint8_t k=0; k <= endCalibration; k++) {
        static float pressure;
        //barometer.getPressure(& pressure);
        calibrate(pressure);
    }
    // Begin serial comms
    Serial.begin(115200);

    // Set up the interrupt pin, it's set as active high, push-pull
    //pinMode(INTERRUPT_PIN, INPUT);
    //attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);

    // Start I^2C
    Wire.begin();
    Wire.setClock(400000); // I2C frequency at 400 kHz
    delay(1000);

    // Start the EM7180 in master mode, no interrupt
    if (!sentral.begin()) {
        while (true) {
            Serial.println(sentral.getErrorString());
        }
    }

    // Get actual gyro rate for conversion to radians
    uint8_t accFs=0; uint16_t gyroFs=0; uint16_t magFs=0;
    sentral.getFullScaleRanges(accFs, gyroFs, magFs);
    gyroAdcToRadians = M_PI * (float)gyroFs / (1<<15) / 180.;  

}

void loop(void)
{
    // get all necessary data
    static float pressure;
    float baroHeight = getAltitude(pressure);

    uint32_t timestamp = micros();

    static float accelData[3];
    static float gyroData[3];
    //getGyrometerAndAccelerometer(gyroData, accelData);

    altitude.estimate(accelData, gyroData, baroHeight, timestamp);

    Serial.print(baroHeight);
    Serial.print(",");
    Serial.print(altitude.getAltitude());
    Serial.print(",");
    Serial.print(altitude.getVerticalVelocity());
    Serial.print(",");
    Serial.println(altitude.getVerticalAcceleration());
}
