
from PyYDLidar.PyYDLidar import YDLidarX4, LaserScan

import time
import matplotlib.pyplot as plt
import math
import threading


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

        #     scan = lidar.getScanData()
        #     for i in range(scan.ranges.shape[0]):
        #         angle = scan.config.min_angle + i * scan.config.ang_increment
        #         dist = scan.ranges[i]
        #     print(scan.ranges)
        #     # print("{}, {}".format(angle, dist))
        # print(lidar._scan_node_count)  # , lidar._scan_node_buf)
        # time.sleep(0.1)
    is_plot = False
    lidar.stopScanning()
