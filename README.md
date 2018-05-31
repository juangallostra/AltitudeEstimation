# altitude-estimation-two-step
A two-step Kalman/Complementary filter for Estimation of Vertical Position using an IMU-Barometer sytem

This work is an implementation of [this paper](http://www.koreascience.or.kr/article/ArticleFullRecord.jsp?cn=HSSHBT_2016_v25n3_202) written in 2016 by Jung Keun Lee. Although the original is in Korean you can find an English version of it [here](https://home.wlu.edu/~levys/TwoStepFilter.pdf) thanks to [Simon D. Levy](http://home.wlu.edu/~levys/). I recommend to have both versions at hand though since there are small typos in some of the equations of the English version.

## Setup

### Hardware

The current setup used for testing consists in the [Butterfly STM32L433 Development Board](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/) plus the [MPU9250 Mini Add-On](https://www.tindie.com/products/onehorse/mpu9250-teensy-3x-add-on-shields/) connected through USB to the computer.

Yo can see it in the images below:

![Setup](https://github.com/juangallostra/altitude-estimation-two-step/blob/master/images/bottom_top.png)

### Software

The program expects the data received through the serial to be in the format:

`ax, ay, az, gx, gy, gz, pressure, timestamp`

where `ax`, `ay` and `az` are measured in g-s, `gz`, `gy` and `gz` in radians per second and `pressure` in millibars. 

**Note**: This project is currently under development.
