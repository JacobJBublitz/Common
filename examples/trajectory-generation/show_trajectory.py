#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt('trajectory.csv', delimiter=',', names=True)

plt.figure(1, figsize=(12, 8))

plt.subplot(231)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Path')
plt.plot(data['x'], data['y'])

plt.subplot(232)
plt.xlabel('Time')
plt.ylabel('Rotation')
plt.title('Rotation')
plt.plot(data['time'], data['rotation'])

plt.subplot(234)
plt.xlabel('Time')
plt.ylabel('Distance')
plt.title('Distance')
plt.plot(data['time'], data['distance'])

plt.subplot(235)
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity')
plt.plot(data['time'], data['velocity'])

plt.subplot(236)
plt.xlabel('Time')
plt.ylabel('Acceleration')
plt.title('Acceleration')
plt.plot(data['time'], data['acceleration'])

plt.suptitle('Trajectory')
plt.show()
