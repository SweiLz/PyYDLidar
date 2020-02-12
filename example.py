
import math
import threading
import time

import matplotlib.pyplot as plt

from PyYDLidar.PyYDLidar import LaserScan, YDLidarX4


def draw():
    global is_plot
    while is_plot:
        plt.figure(1)
        plt.cla()
        plt.ylim(-2, 2)
        plt.xlim(-2, 2)
        plt.scatter(x, y, c='r', s=1)
        plt.pause(0.001)
    plt.close("all")


is_plot = True
x = []
y = []


if __name__ == "__main__":
    lidar = YDLidarX4("/dev/ttyUSB0")

    lidar.getDeviceHealth()
    lidar.startScanning()

    threading.Thread(target=draw).start()

    timeout = 100.0
    timeout_start = time.time()
    while time.time() < timeout_start + timeout:
        lidar.getScanData()
        dx = []
        dy = []
        for i in range(len(lidar.scan.points)):
            point = lidar.scan.points[i]
            dx.append(point.dist * math.cos(point.angle))
            dy.append(point.dist * math.sin(point.angle))

        x, y = dx, dy
        time.sleep(0.1)
        # print(point.angle, point.dist)

    is_plot = False
    lidar.stopScanning()
