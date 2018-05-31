# altitude-estimation-two-step
A two-step Kalman/Complementary filter for Estimation of Vertical Position using an IMU-Barometer sytem

This work is an implementation of [this paper](http://www.koreascience.or.kr/article/ArticleFullRecord.jsp?cn=HSSHBT_2016_v25n3_202) written in 2016 by Jung Keun Lee. Although the original is in Korean you can find an English version of it [here](https://home.wlu.edu/~levys/TwoStepFilter.pdf) thanks to [Simon D. Levy](http://home.wlu.edu/~levys/). I recommend to have both versions at hand though since there are small typos in some of the equations of the English version.

This Python code is a prototype-implementation of the algorithm focused on its validation. The final goal is to implement it in [Hackflight](https://github.com/simondlevy/Hackflight), a simple C++ multirotor flight control firmware for Arduino and simulators.

## Setup

### Hardware

The current setup used for testing consists in the [Butterfly STM32L433 Development Board](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/) plus the [MPU9250 Mini Add-On](https://www.tindie.com/products/onehorse/mpu9250-teensy-3x-add-on-shields/) connected through USB to the computer.

Yo can see it in the images below:

![Setup](https://github.com/juangallostra/altitude-estimation-two-step/blob/master/images/bottom_top.png)

### Software

The Butterfly board is running [Hackflight](https://github.com/simondlevy/Hackflight). You can read how to work with it in [Hackflight's wiki](https://github.com/simondlevy/Hackflight/wiki). Since Hackflight is currently under development, to get this piece of software running you will have to follow a few additional steps.

First of all, you will need the [MS5637 Arduino library](https://github.com/BonaDrone/MS5637), originally developed by [Angus Gratton](https://github.com/projectgus), under your `Arduino/libraries` folder. Yo can simply open a terminal inside the directory and clone the repository.

Secondly, you should switch to BonaDrone's `proxy-data` Hackflight's branch. This branch contains the code (under development) to read data from the baro and also proxy all the required values for the altitude estimation through the serial. This program expects the data received through the serial to be in the format:

`ax, ay, az, gx, gy, gz, pressure, timestamp`

where `ax`, `ay` and `az` are measured in g-s, `gz`, `gy` and `gz` in radians per second and `pressure` in millibars. 

Once this two things have been done it is time to load the firmware to the board and run this code.

**Important**: Since the sensor calibration and bias computation is performed at startup, to get good results it is key to make sure that the board is properly orientated when powered up. The proper orientation is the one shown in the previous image (the one on the right).

## Usage 



**Note**: This project is currently under development.
