# AltitudeEstimation
An arduino library to perform vertical position estimation using an IMU-Barometer system via a two-step Kalman/Complementary filter.

This work is an implementation of the algorithm explained in [this paper](http://www.koreascience.or.kr/article/ArticleFullRecord.jsp?cn=HSSHBT_2016_v25n3_202) written in 2016 by Jung Keun Lee. Although the original is in Korean you can find an English version of it [here](https://home.wlu.edu/~levys/TwoStepFilter.pdf) thanks to [Simon D. Levy](http://home.wlu.edu/~levys/).

**Note**: This readme is not yet complete since this repo has been updated from a Python implementation of the algorithm to an Arduino library. While it is not finished, [here]() you can find the old readme. 

## Setup

### Hardware

The current setup used for testing consists in the [Butterfly STM32L433 Development Board](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/) plus the [MPU9250 Mini Add-On](https://www.tindie.com/products/onehorse/mpu9250-teensy-3x-add-on-shields/) connected through USB to the computer.

Yo can see it in the images below:

![Setup](https://i.imgur.com/XqFxrWS.png)

## Usage


### Available methods

### estimate

Each time this method is called the estimation of the vertical position, velocity and acceleration will be updated. To update the estimation you must provide the latest available readings from the accelerometer (in g-s), the gyrometer (in rad/s) and the baro (in meters) as well as the timestamp in which the readings were obtained. Currently the library assumes the timestamp is measured in milliseconds.

Calling this method will update the estimations and store the results internally but it will not return anything. There are specific methods provided to get the estimated values. 

Method signature:

```cpp
void estimate(float accel[3], float gyro[3], float baroHeight, float timestamp)
```

### getAltitude

This method can be called to obtain the latest vertical position estimation. The estimated altitude is measured in meters.

Method signature:

```cpp
float getAltitude()

```

### getVerticalVelocity

This method can be called to obtain the latest vertical velocity estimation. The estimated velocity is measured in meters per second.

Method signature:

```cpp
float getVerticalVelocity()
```

### getVerticalAcceleration

This method can be called to obtain the latest vertical acceleration estimation. The estimated vertical acceleration is measured in meters per second^2.

Method signature:

```cpp
float getVerticalAcceleration()
```

## Parameter tunning

There are a few parameters that can be tuned to try to achieve higher accuracy. These paremeters are:

1. `sigmaAccel`: standard deviation of the accelerometer
2. `sigmaGyro`: standard deviation of the gyroscope
3. `sigmaBaro`: standard deviation of the barometer
4. `ca`:  constant value for the markov chain acceleration model: a(k) = ca * a(k-1) + e
5. `accelThreshold`: vertical acceleration threshold. If 12 consecutive vertical acceleration values are below the threshold the vertical velocity will be set to 0.

The value of this parameters must be specified when calling the constructor of `AltitudeEstimator`. The order is the one listed above. Below one can see the signature of the constructor:

```cpp
AltitudeEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro, float ca, float accelThreshold)
```

## Results


## Extras