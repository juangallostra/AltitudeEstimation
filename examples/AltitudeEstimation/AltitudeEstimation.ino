/*
   AltitudeEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>
#include "altitude.hpp"

AltitudeEstimator altitude = new AltitudeEstimator();

void setup(void)
{
  
}

void loop(void)
{
  height = altitude.estimate();
  Serial.println(height);
}
