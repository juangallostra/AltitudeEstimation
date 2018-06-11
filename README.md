# altitude-estimation-two-step
A two-step Kalman/Complementary filter for Estimation of Vertical Position using an IMU-Barometer sytem.

![animated](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/results/animated.gif)

**Note**: Although the core of the algorithm works, this project is still under development.

This work is an implementation of [this paper](http://www.koreascience.or.kr/article/ArticleFullRecord.jsp?cn=HSSHBT_2016_v25n3_202) written in 2016 by Jung Keun Lee. Although the original is in Korean you can find an English version of it [here](https://home.wlu.edu/~levys/TwoStepFilter.pdf) thanks to [Simon D. Levy](http://home.wlu.edu/~levys/).

This Python code is a prototype-implementation of the algorithm focused on its validation. The final goal is to implement it in [Hackflight](https://github.com/simondlevy/Hackflight), a simple C++ multirotor flight control firmware for Arduino and simulators.

**UPDATE**: A first implementation of the algorithm already exists in Hackflight. Its current status of can be checked at the [branch](https://github.com/BonaDrone/Hackflight/tree/altitude-hold) `altitude-hold` of BonaDrone's Hackflight fork. Note that it is still under development.

## Requirements

* `numpy`
* `matplotlib` (only if you want to visualize results)
* `pyserial`

## Setup

### Hardware

The current setup used for testing consists in the [Butterfly STM32L433 Development Board](https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/) plus the [MPU9250 Mini Add-On](https://www.tindie.com/products/onehorse/mpu9250-teensy-3x-add-on-shields/) connected through USB to the computer.

Yo can see it in the images below:

![Setup](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/images/bottom_top.png)

### Software

The Butterfly board is running [Hackflight](https://github.com/simondlevy/Hackflight). You can read how to work with it in [Hackflight's wiki](https://github.com/simondlevy/Hackflight/wiki). Since Hackflight is currently under development, to get this piece of software running you will have to follow a few additional steps.

First of all, you will need the [MS5637 Arduino library](https://github.com/BonaDrone/MS5637), originally developed by [Angus Gratton](https://github.com/projectgus), under your `Arduino/libraries` folder. Yo can simply open a terminal inside the directory and clone the repository.

Secondly, you should switch to BonaDrone's `proxy-data` [Hackflight's branch](https://github.com/BonaDrone/Hackflight/tree/proxy-data). This branch contains the code (under development) to read data from the baro and also proxy all the required values for the altitude estimation through the serial. To do so you can add BonaDrone's repo as a new remote and checkout the required branch. This can be done by opening a terminal session inside Hackflight's directory and typing:

`$ git remote add bonadrone https://github.com/BonaDrone/Hackflight.git`

`$ git checkout -b proxy-data bonadrone/proxy-data`

If you are working with different software, this program expects the data received through the serial to be in the format:

`ax, ay, az, gx, gy, gz, pressure, timestamp`

where `ax`, `ay` and `az` are measured in g-s, `gz`, `gy` and `gz` in radians per second and `pressure` in millibars. 

Once this two things have been done it is time to load the firmware to the board and run this code.

**Important**: If using the above mentioned setup, the sensor calibration and bias computation is performed at startup. To get good results it is key to make sure that the board is properly orientated when powered up. The proper orientation is the one shown in the previous image (the one on the right).

## Usage 

My current workflow to use this code, which is not as elegant as it could be is:

0. Check the port being used by the board and modify line 30 of `main.py` accordingly. Do the same with the baudrate (line 31) if you plan on using a different one.

1. Capture the terminal printed values into a log file:

		$ python main.py > log.txt

2. Wait a bit until the calibration is done and then perform the test. When done, force quit the program with `Ctr+C`

3. Open the logfile and remove the calibration rows (until the last column readings are `0.something`)

4. Run `plot_log.py` to get the plot of the data:

		$ python plot_log.py 


## Parameter tunning

There are a few parameters that can be tuned to try to achieve higher accuracy. These paremeters are:

1. `DESIRED_DAMPLING`: Desired sampling period. If the real sampling period is bigger, the data will be oversampled by a linear interpolation of two real consecutive readings.
2. `sigma_accel`: standard deviation of the accelerometer
3. `sigma_gyro`: standard deviation of the gyroscope
4. `sigma_baro`: standard deviation of the barometer
5. `ca`:  constant value for the markov chain acceleration model: a(k) = ca * a(k-1) + e
6. `THRESHOLD`: vertical acceleration threshold. If 12 consecutive vertical acceleration values are below the threshold the vertical velocity will be set to 0.


## Results

Results are in  SI. All distance units are in meters and time units in seconds. This means that acceleration is measured in m/s^2, velocity is measured in m/s and height is measured in m.

### Test 1

This first results are from a test in which the board was lifted with the hand the whole length of the cable that connects it to the computer while being rotated 90 degress around the y-axis, then lowered to half the length of the cable while recovering its original orientation and finally lowered again to the ground.

![results](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/results/log.png)

### Test 2

![results_2](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/results/log_2.png)

### Test 3

![results_3](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/results/log_3.png)

### Test 4

Lift the board to the maximum length of the cable connecting it to the computer and lower it to the ground again 3 times. After that repeat the lift-lower procedure once but this time stopping at an altitude which is roughle the half of the cable before reaching the same altitude as before.

![results_4](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/results/log_4.png)

### Test 5

Lift the board to half the length of the cable. Wait and then go up the full length. Wait and go down to half the length of the cable. Go down to the ground. Wait. Lift to the full cable length and lower to the ground three times without stopping nor at the top or the bottom.

![results_5](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/results/log_5.png)

### Hackflight's test 1

A first and unpolished first version of the algorithm presented in the paper has been implemented in Hakflight. The code can be found at  the [`altitude-hold`](https://github.com/BonaDrone/Hackflight/tree/altitude-hold) branch of BonaDrone's fork. The results of a first test can be observed in the image below, which is a direct screenshot of Arduino's IDE Serial plotter. The color legend is as follows:

* **Blue**: Estimated vertical acceleration (m/s^2).
* **Red**: Estimated vertical velocity (m/s).
* **Green**: Estimated altitude from barometer pressure readings (m).
* **Yellow**: Two step Kalman/Complementary filter estimated altitude (m).


![real_drone_results](https://github.com/juangallostra/AltitudeEstimation/blob/master/extras/results/drone_log_1.png)

