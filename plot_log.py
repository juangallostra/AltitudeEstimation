import numpy as np
import matplotlib.pyplot as plt

with open("log.txt") as f:
	data = f.readlines()

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Altitude estimation")    
ax1.set_xlabel('Sample')
ax1.set_ylabel('Altitude')

ax1.plot(data, c='b', label='data')

leg = ax1.legend()

plt.show()