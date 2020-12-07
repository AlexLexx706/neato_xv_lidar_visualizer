import matplotlib.pyplot as plt
import numpy as np
import time
fig = plt.figure()
ax = fig.add_subplot(111, projection='polar' )
lt = 0

max_points = 360
theta = np.linspace(0, 2 * np.pi, max_points)
ax.set_ylim(0, 20)
ax.set_xlim(0, np.pi * 2)
data = np.random.rand(max_points) * 5.
sc = ax.scatter(theta, data, cmap='hsv')

i = 0
while True:
    data = np.ones(max_points) * i
    i += 0.1
    if i >=  10.:
        i = 0

    sc.set_offsets(np.c_[theta, data])
    plt.draw()
    plt.pause(0.01)

