/*
  StandardDeviations.ino : Arduino example sketch to compute the standard
  deviations of the sensors used for the altitude estimation.
*/

#include <Arduino.h>
#include <math.h>

// Assuming the IMU is an MPU9250 and thr baro a MS5637
#include <MPU9250.h>
#include <MS5637.h>

int iterations = 200;

float getBarometerSigma(int numberOfIterations)
{
   // Initialize barometer
   MS5637 barometer = MS5637();
   barometer.begin();
   // here we will store all pressure readings
   float history[numberOfIterations];
   float meanPressure = 0;
   for (uint8_t index = 0; index < numberOfIterations; index++) {
       float readPressure;
       barometer.getPressure(& readPressure);
       history[index] = readPressure;
       // we will use pressureSum to compute the mean pressure
       meanPressure = meanPressure + readPressure;
   }
   meanPressure = meanPressure / numberOfIterations;
   // Compute standard deviation
   float numerator = 0;
   for (uint8_t index = 0; index < numberOfIterations; index++) {
     numerator = numerator + pow(history[index] - meanPressure, 2);
   }
   return sqrt(numerator / (numberOfIterations - 1));
}

//float getAccelerometerSigma(int numberOfIterations)
//{
//
//}

//float getGyrometerSigma(int numberOfIterations)
//{
//
//}

// Arduino setup and loop functions
void setup(void)
{
  // Begin serial comms
  Serial.begin(115200);
}

void loop(void)
{
    Serial.println("Computing Barometer standard deviation");
    float baroSigma = getBarometerSigma(iterations);
    Serial.print("Barometer standard deviation: ");
    Serial.println(baroSigma);
}
