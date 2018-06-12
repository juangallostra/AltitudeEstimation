/*
   AltitudeEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>
#include "altitude.hpp"

AltitudeEstimator altitude = AltitudeEstimator();

void setup(void)
{

}

void loop(void)
{
  float height = altitude.estimate();
  Serial.println(height);
}
