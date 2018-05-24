# Quick sketch that implements vertical acceleration
# estimation from IMU readings via a Kalman filter
#
# Author: Juan Gallostra
# Date: 16-05.2018

import serial
import time
import numpy as np
import numpy.linalg as la

import matplotlib.pyplot as plt

# Sensor bias - educated guess
GX_BIAS = 0.02
GY_BIAS = 0.01
GZ_BIAS = 0.01

AX_BIAS = 0.1962
AY_BIAS = 0.1962
AZ_BIAS = 0

# standard deviation of sensors - educated guess
sigma_accel = 4.0
sigma_gyro = 2.0
sigma_baro = 2.0

# gravity in m/s^2 
g = 9.81

# more guesses - it has to be less that one for sure (I think)
ca = 0.5

# gain of complementary filter
Kc = np.array([(2*(sigma_accel/sigma_baro))**0.5, sigma_accel/sigma_baro]) 

# Serial parameters
PORT = '/dev/ttyACM0'
BAUDRATE = 9600


def skew(v):
	"""
	Returns the skew symmetric matrix of a vector
	"""
	return np.array([[0,   -v[2],  v[1]],
					 [v[2],   0,  -v[0]],
					 [-v[1], v[0],  0]])


def vector_angle(v1, v2):
    """ 
    Returns the angle in radians between vectors 'v1' and 'v2'    
    """
    return np.arccos(np.dot(v1, v2) / (la.norm(v1) * la.norm(v2)))*180/np.pi


def millibars_to_meters(mb, ground_height=0):
	"""
	Relate pressure to height
	"""
	return 44330*(1 - (mb/1013.25)**0.19) - ground_height


def calibrate(baro, ground_pressure, HISTORY, count):
	"""
	Compute the groun pressure and altitude from a series od readings
	"""
	ground_pressure -= ground_pressure/8
	if len(HISTORY) > count % 48:
		del HISTORY[count % 48]
	HISTORY.insert(count % 48, baro)
	ground_pressure += sum(HISTORY)/48
	if count == 200:
		return True, millibars_to_meters(ground_pressure/8), ground_pressure, HISTORY
	return False, millibars_to_meters(ground_pressure/8), ground_pressure, HISTORY


def get_sensor_data(serial_obj):
	"""
	Get accel, gyro and barometer data from serial
	"""
	raw_data = serial_obj.readline().rstrip().split(",")
	data = map(float, raw_data)
	# split into gyro and accel readings
	accel = np.array(data[:3])*g - np.array([AX_BIAS, AY_BIAS, AZ_BIAS])
	# account for gyro bias
	gyro = np.array(data[3:6]) - np.array([GX_BIAS, GY_BIAS, GZ_BIAS])
	# pressure
	baro = data[-1]
	return accel, gyro, baro


def get_prediction_covariance(v, t, sigma):
	"""
	Get the prediction covariance matrix
	"""
	Sigma = (sigma**2)*np.identity(3) 
	return (-t**2)*skew(v).dot(Sigma).dot(skew(v))


def get_measurement_covariance(ca, a, sigma):
	"""
	Get the measurement covariance matrix
	"""
	Sigma = (sigma**2)*np.identity(3)
	return Sigma + (1.0/3)*(ca**2)*la.norm(a)*np.identity(3)


def predict_state(gyro, z_axis, delta_t):
	"""
	Predict the state evolution of the system one step ahead
	"""
	return (np.identity(3) - delta_t*skew(gyro)).dot(z)


def predict_error_covariance(gyro, z, T, P, sigma_gyro):
	"""
	Project the covariance matrix from the data we have 
	"""
	I = np.identity(3) 
	Q = get_prediction_covariance(z, T, sigma_gyro) # Prediction covariance matrix
	return (I - T*skew(gyro)).dot(P).dot((I - T*skew(gyro)).T) + Q # update 


def update_kalman_gain(P, H, ca, a, sigma_accel):
	"""
	Compute the gain from the predicted error covariance matrix
	and the measurement covariance matrix
	"""
	R = get_measurement_covariance(ca, a, sigma_accel)
	return P.dot(H.T).dot(la.inv(H.dot(P).dot(H.T) + R))

def update_state_with_measurement(z, K, measurement, H):
	"""
	Update the state estimate with the measurement
	"""
	return z + K.dot(measurement - H.dot(z))

def update_error_covariance(P, H, K):
	"""
	Update the error covariance with the calculated gain
	"""
	return (np.identity(3) - K.dot(H)).dot(P) 


# serial communication object
serial_com = serial.Serial(PORT, BAUDRATE)

# Initialize needed variables
prev_time = time.time()
ZUPT_counter = 0
z = np.array([0, 0, 1]) # assume earth and body frame have same orientation
a = np.array([0, 0, g]) # then the only component of the acceleration is g in z
P = np.array([[100, 0, 0],[0, 100, 0],[0, 0, 100]])
H = g*np.identity(3)
v = 0 # vertical velocity

ground_pressure = 0
ground_height = 0
HISTORY = []
calibrated = False
count = 0
# for complementary filter
baro_prev = 0
a_earth_prev = 0
# form kalman filter
z_prev = z

while True:

	# get new sensor data
	accel, gyro, baro = get_sensor_data(serial_com)
	if not calibrated:
		calibrated, ground_height, ground_pressure, HISTORY = calibrate(baro, ground_pressure, HISTORY, count)
		h = 0
		count += 1
	# Calculate sampling time
	curr_time = time.time()
	T = curr_time - prev_time


	# Kalman filter for vertical acceleration estimation

	# Prediction update with data from previous iteration and sensorss
	z = predict_state(gyro, z, T) # State prediction
	z /= la.norm(z)
	P = predict_error_covariance(gyro, z_prev, T, P, sigma_gyro)
	# Measurement update
	K = update_kalman_gain(P, H, ca, a, sigma_accel)
	measurement = accel - ca*a
	z = update_state_with_measurement(z, K, measurement, H)
	P = update_error_covariance(P, H, K)
	z /= la.norm(z)	
	# compute the acceleration from the estimated value of z
	a = accel - g*z
	# print the data for debugging purposes
	# print "angle: " + str(vector_angle(z, np.array([0, 0, 1])))
	# Acceeration in earth reference frame
	#print z
	a_earth = a.dot(z)
	#print accel, a, a_earth


	# Complementary filter for altitude and vertical velocity estimation

	# ZUPT
	if la.norm(a_earth) < 0.1:
		ZUPT_counter += 1
	else:
		ZUPT_counter = 0
	if ZUPT_counter == 12:
		v = 0
		ZUPT_counter = 0

	state = np.array([h, v])
	if baro_prev and a_earth_prev:
		state = np.array([[1, T],[0, 1]]).dot(state) + \
	        	np.array([[1, T/2],[0, 1]]).dot(Kc)*T*(millibars_to_meters(baro_prev, ground_height) - h) + \
	        np.array([T/2, 1])*T*a_earth_prev
	h, v = state
	print h*100
	# complementary filter estimates from values of previous measurements
	baro_prev = baro
	a_earth_prev = a_earth
	z_prev = z
	# Update time of last measurement
	prev_time = curr_time
serial_com.close()
