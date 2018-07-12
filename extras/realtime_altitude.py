#!/usr/bin/env python3
'''
Real-time plot demo using serial input from Arduino altitude-estimation sketch

Dependencies: numpy, matplotlib, https://github.com/simondlevy/RealtimePlotter

Copyright (C) 2018 Simon D. Levy
'''


import serial
from realtime_plot import RealtimePlotter
import numpy as np
from threading import Thread
from sys import argv

# Change these to suit your needs
PORT = '/dev/ttyACM0'
BAUD = 115200

BARO_RANGE         = 1
ALTITUDE_RANGE     = 5
VELOCITY_RANGE     = 1
ACCELERATION_RANGE = 5
NTICKS             = 10

class SerialPlotter(RealtimePlotter):

    def __init__(self):

        ranges = [(-lim,+lim) for lim in [BARO_RANGE, ALTITUDE_RANGE, VELOCITY_RANGE, ACCELERATION_RANGE]]

        RealtimePlotter.__init__(self, 
                ranges, 
                show_yvals=True,
                ylabels=['Barometer', 'Altitude', 'Velocity', 'Acceleration'],
                yticks=[np.linspace(rng[0], rng[1], NTICKS-1) for rng in ranges],
                window_name='Altitude Estimation',
                styles=['b', 'r', 'g', 'y'])

        self.tick = 0
        self.vals = None

    def getValues(self):

         return self.vals

def _update(port, plotter):

    while True:

        plotter.vals = [float(s) for s in port.readline().decode()[:-2].split(',')]

        plotter.tick += 1

if __name__ == '__main__':

    port = argv[1] if len(argv) > 1 else PORT

    try:
        port = serial.Serial(port, BAUD)
    except serial.SerialException:
        print('Unable to open device on port %s' % PORT)
        exit(1)

    plotter = SerialPlotter()

    thread = Thread(target=_update, args = (port, plotter))
    thread.daemon = True
    thread.start()

    plotter.start()
