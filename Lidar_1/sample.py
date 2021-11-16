#https://github.com/nesnes/YDLidarX2_python
import time
from LidarX2 import LidarX2
import numpy as np
import matplotlib.pyplot as plt
lidar = LidarX2("/dev/ttyUSB0")  # Name of the serial port, can be /dev/tty*, COM*, etc.

if not lidar.open():
    print("Cannot open lidar")
    exit(1)

t = time.time()
while time.time() - t < 20:  # Run for 20 seconds
    measures = lidar.getMeasures()  # Get latest lidar measures
    print(measures)

    x = []
    y = []

    for point in measures:
        #print(type(point))
        #print(point.angle)
        if point.distance > 0:
            x.append(point.distance*np.sin(point.angle*3.1415/180))
            y.append(point.distance*np.cos(point.angle*3.1415/180))
    plt.clf()
    plt.scatter(x, y)
    plt.pause(.1)

    time.sleep(0.1)

lidar.close()
