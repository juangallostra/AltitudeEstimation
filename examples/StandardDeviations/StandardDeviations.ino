/*
  StandardDeviations.ino : Arduino example sketch to compute the standard
  deviations of the sensors used for the altitude estimation.
*/

#include <Arduino.h>
#include <ArduinoTransfer.h>
#include <Wire.h>
#include <math.h>

// Assuming the IMU is an MPU9250 and thr baro a MS5637
#include <MPU9250.h>
#include <MS5637.h>

// Number of readings from which standard deviations will be computed
uint16_t iterations = 1000;

// Function to compute barometer standard deviations
float getBarometerSigma(uint16_t numberOfIterations)
{
   // Initialize barometer
   MS5637 barometer = MS5637();
   barometer.begin();
   // here we will store all pressure readings
   float history[numberOfIterations];
   float meanPressure = 0;
   for (uint16_t index = 0; index < numberOfIterations; index++) {
       float readPressure;
       barometer.getPressure(& readPressure);
       history[index] = readPressure;
       // we will use pressureSum to compute the mean pressure
       meanPressure += readPressure;
   }
   meanPressure /= numberOfIterations;
   // Compute standard deviation
   float numerator = 0;
   for (uint16_t index = 0; index < numberOfIterations; index++) {
     numerator += pow(history[index] - meanPressure, 2);
   }
   return sqrt(numerator / (numberOfIterations - 1));
}

// Acelerometer anf Gyrometer helper methods and variables

// Sensor full-scale settings
const Ascale_t ASCALE = AFS_8G;
const Gscale_t GSCALE = GFS_2000DPS;
const Mscale_t MSCALE = MFS_16BITS;
const Mmode_t MMODE = M_100Hz;
// SAMPLE_RATE_DIVISOR: (1 + SAMPLE_RATE_DIVISOR) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
// SAMPLE_RATE_DIVISOR = 0 means 1 kHz sample rate for both accel and gyro, 4 means 200 Hz, etc.
const uint8_t SAMPLE_RATE_DIVISOR = 0;
// MPU9250 add-on board has interrupt on Butterfly pin 8
const uint8_t INTERRUPT_PIN = 8;
// Create byte-transfer objects for Arduino I^2C
ArduinoI2C mpu = ArduinoI2C(MPU9250::MPU9250_ADDRESS);
ArduinoI2C mag = ArduinoI2C(MPU9250::AK8963_ADDRESS);

// Use the MPU9250 in pass-through mode
MPU9250Passthru imu = MPU9250Passthru(&mpu, &mag);;
// Store imu data
int16_t imuData[7] = {0,0,0,0,0,0,0};
// For scaling to normal units (accelerometer G's, gyrometer rad/sec, magnetometer mGauss)
float aRes;
float gRes;
float mRes;
// We compute these at startup
float gyroBias[3]  = {0,0,0};
float accelBias[3] = {0,0,0};
// flag for when new data is received
bool gotNewData = false;

static void interruptHandler()
{
    gotNewData = true;
}

// Raw analog-to-digital values converted to radians per second
float adc2rad(int16_t adc, float bias)
{
    return (adc * gRes - bias) * M_PI / 180;
}

void getGyrometerAndAccelerometer(float gyro[3], float accel[3])
{
    if (gotNewData) {

        gotNewData = false;

        if (imu.checkNewAccelGyroData()) {

            imu.readMPU9250Data(imuData);

            // Convert the accleration value into g's
            float ax = imuData[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            float ay = imuData[1]*aRes - accelBias[1];
            float az = imuData[2]*aRes - accelBias[2];

            // Convert the gyro value into degrees per second
            float gx = adc2rad(imuData[4], gyroBias[0]);
            float gy = adc2rad(imuData[5], gyroBias[1]);
            float gz = adc2rad(imuData[6], gyroBias[2]);

            // Copy gyro values back out
            gyro[0] = gx;
            gyro[1] = gy;
            gyro[2] = gz;
            // and acceleration values
            accel[0] = ax;
            accel[1] = ay;
            accel[2] = az;

        } // if (imu.checkNewAccelGyroData())

    } // if gotNewData

}

// Function to compute accelerometer and gyrometer standard deviations
void getAccelAndGyroSigmas(double* sigmaAccel, double* sigmaGyro, uint16_t numberOfIterations)
{
    // here we will store each accel axis' readings
    float accelHistoryX[numberOfIterations];
    float accelHistoryY[numberOfIterations];
    float accelHistoryZ[numberOfIterations];
    // here we will store each gyro axis' readings
    float gyroHistoryX[numberOfIterations];
    float gyroHistoryY[numberOfIterations];
    float gyroHistoryZ[numberOfIterations];
    float meanAccelX = 0;
    float meanAccelY = 0;
    float meanAccelZ = 0;
    float meanGyroX = 0;
    float meanGyroY = 0;
    float meanGyroZ = 0;

    for (uint16_t index = 0; index < numberOfIterations; index++) {
        float readGyro[3];
        float readAccel[3];
        getGyrometerAndAccelerometer(readGyro, readAccel);
        // store gyro readings
        gyroHistoryX[index] = readGyro[0];
        gyroHistoryY[index] = readGyro[1];
        gyroHistoryZ[index] = readGyro[2];
        // store accel readings
        accelHistoryX[index] = readAccel[0];
        accelHistoryY[index] = readAccel[1];
        accelHistoryZ[index] = readAccel[2];
        // increase mean sums
        meanGyroX += readGyro[0];
        meanGyroY += readGyro[1];
        meanGyroZ += readGyro[2];

        meanAccelX += readAccel[0];
        meanAccelY += readAccel[1];
        meanAccelZ += readAccel[2];
    }
    // Compute means
    meanGyroX /= numberOfIterations;
    meanGyroY /= numberOfIterations;
    meanGyroZ /= numberOfIterations;

    meanAccelX /= numberOfIterations;
    meanAccelY /= numberOfIterations;
    meanAccelZ /= numberOfIterations;

    // Compute standard deviations
    double numeratorGyroX = 0;
    double numeratorGyroY = 0;
    double numeratorGyroZ = 0;

    double numeratorAccelX = 0;
    double numeratorAccelY = 0;
    double numeratorAccelZ = 0;

    for (uint16_t index = 0; index < numberOfIterations; index++) {
      numeratorGyroX += pow(gyroHistoryX[index] - meanGyroX, 2);
      numeratorGyroY += pow(gyroHistoryY[index] - meanGyroY, 2);
      numeratorGyroZ += pow(gyroHistoryZ[index] - meanGyroZ, 2);

      numeratorAccelX += pow(accelHistoryX[index] - meanAccelX, 2);
      numeratorAccelY += pow(accelHistoryY[index] - meanAccelY, 2);
      numeratorAccelZ += pow(accelHistoryZ[index] - meanAccelZ, 2);
    }
    // Now, to compute on single standard deviation value for each
    // sensor, Gyro and Accel, we will take the maximum of the standard
    // deviations of each axis.
    double gyroSigmaX = sqrt(numeratorGyroX / (numberOfIterations - 1));
    double gyroSigmaY = sqrt(numeratorGyroY / (numberOfIterations - 1));
    double gyroSigmaZ = sqrt(numeratorGyroZ / (numberOfIterations - 1));

    double accelSigmaX = sqrt(numeratorAccelX / (numberOfIterations - 1));
    double accelSigmaY = sqrt(numeratorAccelY / (numberOfIterations - 1));
    double accelSigmaZ = sqrt(numeratorAccelZ / (numberOfIterations - 1));

    double tmp  = max(gyroSigmaX, gyroSigmaY);
    *sigmaGyro =  max(tmp, gyroSigmaZ);
    tmp  = max(accelSigmaX, accelSigmaY);
    *sigmaAccel = max(tmp, accelSigmaZ);

}

// Arduino setup and loop functions
void setup(void)
{
    // Begin serial comms
    Serial.begin(115200);
    // Set up the interrupt pin, it's set as active high, push-pull
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);

    // Start I^2C
    Wire.begin();
    Wire.setClock(400000); // I2C frequency at 400 kHz
    delay(1000);

    // Reset the MPU9250
    imu.resetMPU9250();

    // get sensor resolutions, only need to do this once
    aRes = imu.getAres(ASCALE);
    gRes = imu.getGres(GSCALE);
    mRes = imu.getMres(MSCALE);

    // Calibrate gyro and accelerometers, load biases in bias registers
    imu.calibrateMPU9250(gyroBias, accelBias);

    // Initialize the MPU9250
    imu.initMPU9250(ASCALE, GSCALE, SAMPLE_RATE_DIVISOR);
}

void loop(void)
{
    Serial.println("Computing Barometer standard deviation");
    float baroSigma = getBarometerSigma(iterations);
    Serial.print("Barometer standard deviation: ");
    Serial.println(baroSigma, 15);
    Serial.println("Computing Accelerometer and Gyrometer standard deviations");
    double accelSigma;
    double gyroSigma;
    getAccelAndGyroSigmas(&accelSigma, &gyroSigma, iterations);
    Serial.print("Accelerometer standard deviation: ");
    Serial.println(accelSigma, 15);
    Serial.print("Gyrometer standard deviation: ");
    Serial.println(gyroSigma, 15);
}
