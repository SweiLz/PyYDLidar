import threading
import time
from math import atan, pi

import numpy as np
from serial import Serial

from struct import pack, unpack


# class Lidar:
#     RESULT_OK = 0
#     RESULT_TIMEOUT = -1
#     RESULT_FAIL = -2

#     DEFAULT_TIMEOUT = 2000

#     cmd_stop = 0x65
#     cmd_scan = 0x60
#     cmd_force_scan = 0x61
#     cmd_reset = 0x80
#     cmd_force_stop = 0x00
#     cmd_get_eai = 0x55
#     cmd_get_device_info = 0x90
#     cmd_get_device_health = 0x92
#     ans_type_devinfo = 0x4
#     and_type_devhealth = 0x6
#     cmd_sync_byte = 0xA5
#     cmdflag_has_payload = 0x80
#     ans_sync_byte1 = 0xA5
#     ans_sync_byte2 = 0x5A
#     ans_type_measurement = 0x81
#     resp_measurement_syncbit = (0x1 << 0)
#     resp_measurement_quality_shift = 2
#     resp_measurement_sync_quality_shift = 8
#     resp_measurement_checkbit = (0x1 << 0)
#     resp_measurement_angle_shift = 1
#     resp_measurement_angle_sample_shift = 8
#     resp_measurement_distance_shift = 2
#     resp_measurement_distance_half_shift = 1

class LaserScan:
    class LaserPoint:
        angle = 0.0  # lidar angle
        dist = 0.0  # lidar range
        intensity = 0.0  # lidar intensity

    class LaserConfig:
        min_angle = -pi  # Start angle for the laser scan [rad]
        max_angle = pi  # Stop angle for the laser scan [rad]
        ang_increment = None  # Scan resolution [rad]
        time_increment = None  # Scan resolution [s]
        scan_time = None  # Time between scans
        min_range = 0.15  # Minimum range [m]
        max_range = 10.0  # Maximum range [m]

    stamp = None  # System time when first range was measured [ns]
    points = []  # Array of lidar points
    config = LaserConfig()  # Configuration of scan


class YDLidarX4:
    def __init__(self, port):
        self._port = port
        self._baudrate = 128000
        self._isConnected = False
        self._isScanning = False
        try:
            self._serial = Serial(self._port, self._baudrate, timeout=2.0)
            self._isConnected = True
            self._serial.reset_input_buffer()
            self._serial.write([0xA5, 0x00])
            self._serial.write([0xA5, 0x65])
            self._serial.setDTR(0)
            self._serial.flush()

        except Exception as e:
            print("Cannot open port {}".format(self._port))
            return

        self.thread = threading.Thread(target=self.cacheScanData)

        self._package_sample_index = 0
        self._package_type = 0
        self._package_checksum_result = False
        self._package_num = 0
        self._package_sample_distance = []
        self._package_interval_sample_angle = 0
        self._package_first_sample_angle = 0

        self._scan_node_buf = None
        self._scan_node_count = 0

        self.scan = LaserScan()
        self.scan.config.min_angle = -pi
        self.scan.config.max_angle = pi
        self.scan.config.min_range = 0.1
        self.scan.config.max_range = 10.0
        self.scan.points.clear()

    def cacheScanData(self):
        index = 0
        count = 128
        local_scan = np.zeros((3600, 3), dtype=int)
        scan_count = 0
        while self._isScanning:
            # try:
            local_buf, count = self._waitScanData(count)
            # except Exception as e:
            #     print(e)
            #     break

            for pos in range(count):
                if local_buf[pos][0] & (0x1 << 0):
                    if local_scan[0][0] & (0x1 << 0):
                        self._scan_node_buf = local_scan
                        self._scan_node_count = scan_count
                    scan_count = 0
                local_scan[scan_count] = local_buf[pos]
                scan_count += 1
                if scan_count == local_scan.shape[0]:
                    scan_count -= 1

        self._isScanning = False
        # print("Break")

    def _waitScanData(self, count):
        nodebuffer = np.zeros((count, 3), dtype=int)
        recvNodeCount = 0
        while recvNodeCount < count:
            node = self._waitPackage()
            nodebuffer[recvNodeCount] = node
            recvNodeCount += 1

            if node[0] & (0x1 << 0):
                break

        count = recvNodeCount
        return nodebuffer, count

    def _waitPackage(self):
        node = np.array([0, 0, 0], dtype=int)

        recvPos = 0
        LastSampleAngle = 0
        scan_frequence = 0
        packageBuffer = []
        if self._package_sample_index == 0:
            recvPos = 0
            CheckSumCal = 0
            CheckSum = 0
            SampleNumlAndCTCal = 0
            LastSampleAngleCal = 0
            while recvPos != 10:
                currentByte = ord(self._serial.read())
                if recvPos == 0:
                    if currentByte == 0xAA:
                        pass
                    else:
                        continue
                elif recvPos == 1:
                    CheckSumCal = 0x55AA
                    if currentByte == 0x55:
                        pass
                    else:
                        recvPos = 0
                        continue
                elif recvPos == 2:
                    SampleNumlAndCTCal = currentByte
                    self._package_type = currentByte & 0x01
                    if self._package_type == 0:
                        pass
                    elif self._package_type == 1:
                        scan_frequence = (currentByte & 0xFE) >> 1
                    else:
                        recvPos = 0
                        continue
                elif recvPos == 3:
                    SampleNumlAndCTCal += (currentByte * 0x100)
                    self._package_num = currentByte
                elif recvPos == 4:
                    if currentByte & (0x1 << 0):
                        self._package_first_sample_angle = currentByte
                    else:
                        recvPos = 0
                        continue
                elif recvPos == 5:
                    self._package_first_sample_angle += currentByte * 0x100
                    CheckSumCal ^= self._package_first_sample_angle
                    self._package_first_sample_angle = self._package_first_sample_angle >> 1
                elif recvPos == 6:
                    if currentByte & (0x1 << 0):
                        LastSampleAngle = currentByte
                    else:
                        recvPos = 0
                        continue
                elif recvPos == 7:
                    LastSampleAngle = currentByte * 0x100 + LastSampleAngle
                    LastSampleAngleCal = LastSampleAngle
                    LastSampleAngle = LastSampleAngle >> 1

                    if self._package_num == 1:
                        self._package_interval_sample_angle = 0

                    else:
                        if LastSampleAngle < self._package_first_sample_angle:
                            if (self._package_first_sample_angle > 270 * 64) and (LastSampleAngle < 90*64):
                                self._package_interval_sample_angle = float(
                                    (360 * 64 + LastSampleAngle - self._package_first_sample_angle) / (self._package_num - 1))
                        else:
                            self._package_interval_sample_angle = float(
                                (LastSampleAngle - self._package_first_sample_angle)/(self._package_num-1))

                elif recvPos == 8:
                    CheckSum = currentByte
                elif recvPos == 9:
                    CheckSum += (currentByte*0x100)

                recvPos += 1
                packageBuffer.append(currentByte)

            Valu8Tou16 = 0
            self._package_sample_distance.clear()
            # recvBuffer = list(self._serial.read(self._package_num * 2))
            for i in range(self._package_num * 2):
                currentByte = ord(self._serial.read())
                if i % 2 == 1:
                    Valu8Tou16 += currentByte * 0x100
                    CheckSumCal ^= Valu8Tou16
                    self._package_sample_distance.append(Valu8Tou16)
                else:
                    Valu8Tou16 = currentByte

            CheckSumCal ^= SampleNumlAndCTCal
            CheckSumCal ^= LastSampleAngleCal

            if CheckSumCal != CheckSum:
                self._package_checksum_result = False
            else:
                self._package_checksum_result = True
            # print(self._package_checksum_result, packageBuffer)

        if self._package_type == 0:
            node[0] = 2
        else:
            node[0] = 1

        sync_quality = 10

        if self._package_checksum_result:
            node[1] = self._package_sample_distance[self._package_sample_index]
            sync_quality = (
                0xFC | self._package_sample_distance[self._package_sample_index] & 0x0003) << 2

            angleCorrect = self._AngleCorr(node[1])
            sampleAngle = self._package_interval_sample_angle * self._package_sample_index
            if (self._package_first_sample_angle + sampleAngle + angleCorrect) < 0:
                node[2] = (int(
                    (self._package_first_sample_angle + sampleAngle + angleCorrect + 23040)) << 1) + (0x1 << 0)

            else:
                if (self._package_first_sample_angle + sampleAngle + angleCorrect) > 23040:
                    node[2] = (int(
                        (self._package_first_sample_angle + sampleAngle + angleCorrect - 23040)) << 1) + (0x1 << 0)
                else:
                    node[2] = (int(
                        (self._package_first_sample_angle + sampleAngle + angleCorrect)) << 1) + (0x1 << 0)
        else:

            node[0] = 2
            node[1] = 0
            node[2] = (0x1 << 0)

        self._package_sample_index += 1
        if self._package_sample_index >= self._package_num:
            self._package_sample_index = 0
            self._package_checksum_result = False
        return node

    @classmethod
    def _AngleCorr(cls, dist):
        if dist == 0:
            return 0
        else:
            return int((atan(((21.8 * (155.3 - (dist / 4.0))) / 155.3) / (dist / 4.0)) * 180.0/pi)*64.0)

    @classmethod
    def _NormalizeAngle(cls, angle):
        a = ((angle % (2.0*pi))+2.0*pi) % (2.0*pi)
        if a > pi:
            a -= 2.0 * pi
        return a

    def getScanData(self):
        global_nodes = self._scan_node_buf
        all_nodes_counts = self._scan_node_count
        self.scan.points.clear()
        self.scan.config.ang_increment = (
            self.scan.config.max_angle - self.scan.config.min_angle) / (all_nodes_counts - 1)

        # print(all_nodes_counts)
        for i in range(all_nodes_counts):
            # print(global_nodes[i][2], global_nodes[i][1])
            angle = float((global_nodes[i][2] >> 1)/64) * pi / 180.0
            distance = float(global_nodes[i][1]) / 4000.0
            angle = 2*pi - angle
            angle = self._NormalizeAngle(angle)

            if not (distance >= self.scan.config.min_range and distance <= self.scan.config.max_range):
                distance = 0
            if angle >= self.scan.config.min_angle and angle <= self.scan.config.max_angle:
                point = LaserScan.LaserPoint()
                point.angle = angle
                point.dist = distance
                point.intensity = 0
                self.scan.points.append(point)

    def startScanning(self):
        if not self._isConnected:
            return
        self._serial.setDTR(1)
        self._serial.flush()
        self._serial.write([0xA5, 0x60])
        time.sleep(0.1)
        # print(self._serial.inWaiting())
        lidar_ans_header = unpack("<BBhhB", self._serial.read(7))
        # print(lidar_ans_header)

        self._isScanning = True
        self.thread.start()

    def stopScanning(self):
        if not self._isConnected:
            return
        self._serial.setDTR(0)
        self._serial.flush()
        self._isScanning = False

    def getDeviceHealth(self):
        self._serial.reset_input_buffer()
        self._serial.write([0xA5, 0x92])
        time.sleep(0.1)
        lidar_ans_header = unpack("<BBHHB", self._serial.read(7))
        device_health = unpack("<BH", self._serial.read(lidar_ans_header[2]))
        # print(lidar_ans_header)
        # print(device_health)
