import threading
import time
from math import atan, pi

import numpy as np
from serial import Serial


class Lidar:
    RESULT_OK = 0
    RESULT_TIMEOUT = -1
    RESULT_FAIL = -2

    DEFAULT_TIMEOUT = 2000

    cmd_stop = 0x65
    cmd_scan = 0x60
    cmd_force_scan = 0x61
    cmd_reset = 0x80
    cmd_force_stop = 0x00
    cmd_get_eai = 0x55
    cmd_get_device_info = 0x90
    cmd_get_device_health = 0x92
    ans_type_devinfo = 0x4
    and_type_devhealth = 0x6
    cmd_sync_byte = 0xA5
    cmdflag_has_payload = 0x80
    ans_sync_byte1 = 0xA5
    ans_sync_byte2 = 0x5A
    ans_type_measurement = 0x81
    resp_measurement_syncbit = (0x1 << 0)
    resp_measurement_quality_shift = 2
    resp_measurement_sync_quality_shift = 8
    resp_measurement_checkbit = (0x1 << 0)
    resp_measurement_angle_shift = 1
    resp_measurement_angle_sample_shift = 8
    resp_measurement_distance_shift = 2
    resp_measurement_distance_half_shift = 1


class LaserScan:
    class LaserConfig:
        min_angle = -pi  # Start angle for the laser scan [rad]
        max_angle = pi  # Stop angle for the laser scan [rad]
        ang_increment = None  # Scan resolution [rad]
        time_increment = None  # Scan resolution [s]
        scan_time = None  # Time between scans
        min_range = 0.15  # Minimum range [m]
        max_range = 10.0  # Maximum range [m]
        range_res = None  # Range resolution [m]
    ranges = []  # Array of ranges
    intensities = []  # Array of intensities
    self_time_stamp = None  # Self reported time stamp [ns]
    system_time_stamp = None  # System time when first range was measured [ns]
    config = LaserConfig()


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

        self._scan = LaserScan()
        self.thread = threading.Thread(target=self.cacheScanData)

    def cacheScanData(self):
        index = 0
        self._scan.ranges = np.zeros(20)
        # print(self.scan.ranges.shape)
        while self._isScanning:
            for pos in range(self._scan.ranges.shape[0]):
                self._scan.ranges[pos] = pos
                pass

    def getScanData(self):
        nodes = self._scan

        count = nodes.ranges.shape[0]
        all_nodes_counts = count
        each_angle = 360.0 / all_nodes_counts

        angle_compensate_nodes = np.zeros((all_nodes_counts, 2), dtype=int)

        for i in range(all_nodes_counts):
            if nodes[i, 0] != 0:
                angle = (nodes[i, 1] >>
                         Lidar.resp_measurement_angle_shift)/64.0

                inter = int(angle/each_angle)
                angle_pre = angle - inter * each_angle
                angle_next = (inter+1)*each_angle - angle

                if angle_pre < angle_next:
                    if inter < all_nodes_counts:
                        angle_compensate_nodes[inter] = nodes[i]
                else:
                    if inter < all_nodes_counts - 1:
                        angle_compensate_nodes[inter+1] = nodes[i]

        diff_angle = nodes.config.max_angle - nodes.config.min_angle
        counts = int(all_nodes_counts * (diff_angle / (2*pi)))
        angle_start = int(pi + nodes.config.min_angle)
        node_start = int(all_nodes_counts * (angle_start / (2*pi)))

        nodes.ranges = np.zeros(counts)

        index = 0
        for i in range(all_nodes_counts):
            dist_range = angle_compensate_nodes[i, 0] / 4000

            if i < all_nodes_counts // 2:
                index = all_nodes_counts // 2 - 1 - i
            else:
                index = all_nodes_counts - 1 - (i-all_nodes_counts//2)

            if dist_range > nodes.config.max_range or dist_range < nodes.config.min_range:
                dist_range = 0.0

            pos = index - node_start
            if 0 <= pos and pos < counts:
                scan.ranges[pos] = dist_range

        if diff_angle == 2*pi:
            nodes.config.ang_increment = diff_angle / counts
        else:
            nodes.config.ang_increment = diff_angle / (counts - 1)

        # for i in range(0, self.scan.ranges.shape[0], 3):
            # pass
        # for i in range(20):
        #     angle = self.scan.config.min_angle + i * self.scan.config.ang_increment
        #     dist = self.scan.ranges[i]
        #     print("{}: {}".format(angle, dist))
        # print("\n\n")

        return nodes

    def startScanning(self):
        if not self._isConnected:
            return
        self._serial.setDTR(1)
        self._serial.flush()
        self._serial.write([0xA5, 0x60])
        time.sleep(0.1)
        header = list(self._serial.read(7))  # read lidar_ans_header
        # print(header)
        self._isScanning = True
        self.thread.start()

    def stopScanning(self):
        if not self._isConnected:
            return
        self._serial.setDTR(0)
        self._serial.flush()
        self._isScanning = False


class YDLidarX42:
    def __init__(self, port):
        self._port = port
        self._baudrate = 128000

        self.scan = LaserScan()

        self.scan.config.min_angle = -pi
        self.scan.config.max_angle = pi
        self.scan.config.min_range = 0.25
        self.scan.config.max_range = 10.0

        self._intensities = False
        self._auto_reconnect = True
        self._resolution_fixed = True
        self._reversion = False
        self._low_exposure = False
        self._samp_rate = 4
        self._frequency = 7

        self._node_counts = 720

        self._each_angle = 0.5

        self._isConnect = False
        self._isScanning = False

        self.device_info = {
            "Model": None,
            "Firmware version": None,
            "Hardware version": None,
            "Serial number": None
        }

        self.device_health = {
            "Status": None,
            "Error code": None
        }

        self.thread = threading.Thread(target=self.cacheScanData)
        self.laser = LaserScan()
        self.count = 3600
        self.scan_node_buf = np.zeros((self.count, 2), dtype=int)

        self._package_sample_index = 0

    def initialize(self):
        try:
            if not self._isConnect:
                self._serial = Serial(self._port, self._baudrate, timeout=2.0)
                self._isConnect = True
                self._serial.reset_input_buffer()
                self._serial.write([Lidar.cmd_sync_byte, Lidar.cmd_force_stop])
                self._serial.write([Lidar.cmd_sync_byte, Lidar.cmd_stop])
                self.clearDTR()
                # self.setDTR()

            else:
                raise Exception("Already connected")

            if self._isScanning:
                return True
            else:
                if not self.getDeviceHealth():
                    return False
                if not self.getDeviceInfo():
                    return False

        except Exception as e:
            print(e)
            return False

    def startScan(self):
        self._serial.reset_input_buffer()
        self._serial.write([Lidar.cmd_sync_byte, Lidar.cmd_force_stop])
        self._serial.write([Lidar.cmd_sync_byte, Lidar.cmd_stop])

        m_pointTime = 1e9 / 5000
        self.setDTR()
        self._serial.write([Lidar.cmd_sync_byte, Lidar.cmd_scan])
        time.sleep(0.1)
        header = list(self._serial.read(7))  # read lidar_ans_header
        # data = list(self._serial.read(10))  # read data
        print(header)
        # print(data)
        self._isScanning = True
        self.thread.start()

    def stopScan(self):
        self._isScanning = False
        self.thread.join()

    def getDeviceHealth(self):
        self._serial.reset_input_buffer()
        self._serial.write([Lidar.cmd_sync_byte, Lidar.cmd_get_device_health])
        time.sleep(0.1)
        header = list(self._serial.read(7))  # read lidar_ans_header
        data = list(self._serial.read(header[2]))  # read data

        if not any(data):
            return True
        return False

    def getDeviceInfo(self):

        self._serial.reset_input_buffer()
        self._serial.write([Lidar.cmd_sync_byte, Lidar.cmd_get_device_info])
        time.sleep(0.1)
        header = list(self._serial.read(7))  # read lidar_ans_header
        data = list(self._serial.read(header[2]))  # read data

        if data[0] == 6:
            self.device_info["Model"] = "X4"

        ver = int.from_bytes(data[1:3], byteorder='little', signed=False)
        self.device_info["Firmware version"] = "{}.{}.{}".format(
            ver >> 8, (ver & 0xff)//10, (ver & 0xff) % 10)
        self.device_info["Hardware version"] = str(data[3])
        self.device_info["Serial number"] = "".join(map(str, data[4:]))

        print(self.device_info)
        return True

    @classmethod
    def _AngleCorr(cls, dist):
        if dist == 0:
            return 0
        else:
            return int((atan(((21.8 * (155.3 - (dist / 4.0))) / 155.3) / (dist / 4.0)) * 180.0/pi)*64.0)

    def waitScanData(self, nodebuffer, count):
        if not self._isConnect:
            count = 0

        recvNodeCount = 0
        while recvNodeCount < count:
            node = self.waitPackage()
            nodebuffer[recvNodeCount] = node
            recvNodeCount += 1
            if recvNodeCount == count:
                break

        return nodebuffer, count

    def waitPackage(self):
        node = np.array([0, Lidar.resp_measurement_checkbit], dtype=int)

        packageSampleDistance = []

        recvPos = 0
        recvBuffer = []
        packageBuffer = []
        CheckSum = 0
        CheckSumCal = 0
        CheckSumResult = False
        SampleNumlAndCTCal = 0
        LastSampleAngleCal = 0
        package_sample_num = 0
        FirstSampleAngle = 0
        LastSampleAngle = 0
        IntervalSampleAngle = 0
        package_type = 0

        if self._package_sample_index == 0:
            recvPos = 0
            while self._isScanning:
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
                    package_type = currentByte & 0x01
                    if package_type == 0 or package_type == 1:
                        if package_type == 1:
                            scan_frequence = (currentByte & 0xFE) >> 1
                    else:
                        recvPos = 0
                        continue
                elif recvPos == 3:
                    SampleNumlAndCTCal += (currentByte * 0x100)
                    package_sample_num = currentByte
                elif recvPos == 4:
                    if currentByte & Lidar.resp_measurement_checkbit:
                        FirstSampleAngle = currentByte
                    else:
                        recvPos = 0
                        continue
                elif recvPos == 5:
                    FirstSampleAngle += currentByte * 0x100
                    CheckSumCal ^= FirstSampleAngle
                    FirstSampleAngle = FirstSampleAngle >> 1
                elif recvPos == 6:
                    if currentByte & Lidar.resp_measurement_checkbit:
                        LastSampleAngle = currentByte
                    else:
                        recvPos = 0
                        continue
                elif recvPos == 7:
                    LastSampleAngle = currentByte * 0x100 + LastSampleAngle
                    LastSampleAngleCal = LastSampleAngle
                    LastSampleAngle = LastSampleAngle >> 1

                    if package_sample_num == 1:
                        IntervalSampleAngle = 0
                    else:
                        if LastSampleAngle < FirstSampleAngle:
                            if (FirstSampleAngle >= 180 * 64) and (LastSampleAngle <= 180*64):
                                IntervalSampleAngle = float(
                                    (360 * 64 + LastSampleAngle - FirstSampleAngle) / (package_sample_num - 1))
                            else:
                                if FirstSampleAngle > 360:
                                    IntervalSampleAngle = float(
                                        LastSampleAngle-FirstSampleAngle)/(package_sample_num - 1)
                                else:
                                    temp = FirstSampleAngle
                                    FirstSampleAngle = LastSampleAngle
                                    LastSampleAngle = temp
                                    IntervalSampleAngle = float(
                                        (LastSampleAngle - FirstSampleAngle)/(package_sample_num-1))
                        else:
                            IntervalSampleAngle = float(
                                (LastSampleAngle - FirstSampleAngle)/(package_sample_num-1))

                        IntervalSampleAngle_LastPackage = IntervalSampleAngle

                elif recvPos == 8:
                    CheckSum = currentByte
                elif recvPos == 9:
                    CheckSum += (currentByte*0x100)

                packageBuffer.append(currentByte)
                recvPos += 1
                if recvPos == 10:
                    package_recvPos = recvPos
                    break

            if recvPos == 10:
                recvPos = 0
                packageSampleDistance.clear()
                inComingByte = self._serial.inWaiting()
                recvBuffer = list(self._serial.read(inComingByte))
                Valu8Tou16 = 0
                for i in range(inComingByte):
                    if recvPos % 2 == 1:
                        Valu8Tou16 += recvBuffer[i] * 0x100
                        CheckSumCal ^= Valu8Tou16
                        packageSampleDistance.append(Valu8Tou16)
                    else:
                        Valu8Tou16 = recvBuffer[i]
                    packageBuffer.append(recvBuffer[i])
                    recvPos += 1
                if package_sample_num * 2 == recvPos:
                    package_recvPos += recvPos
            else:
                recvBuffer.clear()

            CheckSumCal ^= SampleNumlAndCTCal
            CheckSumCal ^= LastSampleAngleCal

            if CheckSumCal != CheckSum:
                CheckSumResult = False
            else:
                CheckSumResult = True

        sync_flag = 0
        if package_type == 0:
            sync_flag = 2
        else:
            sync_flag = 1

        sync_quality = 10

        if CheckSumResult and recvBuffer != []:
            node[0] = packageSampleDistance[self._package_sample_index]
            AngleCorrectForDistance = self._AngleCorr(node[0])

            temp = FirstSampleAngle + IntervalSampleAngle * \
                self._package_sample_index + AngleCorrectForDistance
            if temp < 0:
                node[1] = (int(temp + 360 * 64) << 1) + \
                    Lidar.resp_measurement_checkbit
            else:
                if temp > 360 * 64:
                    node[1] = (int(temp - 360*64) << 1) + \
                        Lidar.resp_measurement_checkbit
                else:
                    node[1] = (int(temp) << 1)+Lidar.resp_measurement_checkbit
        else:
            sync_flag = 2
            sync_quality = 10

        self._package_sample_index += 1
        if self._package_sample_index >= package_sample_num:
            self._package_sample_index = 0

        return node

    def cacheScanData(self):
        count = 128
        local_buff = np.zeros((count, 2), dtype=int)
        local_scan = np.zeros((3600, 2), dtype=int)
        scan_count = 0

        while self._isScanning:
            local_buff, count = self.waitScanData(local_buff, count)
            print(local_buff)

        # package_sample_index = 0
        # package_sample_num = 0
        # recvPos = 0
        # packageBuffer = []
        # CheckSumCal = 0x55AA
        # CheckSum = 0

        # SampleNumlAndCTCal = 0
        # LastSampleAngleCal = 0
        # FirstSampleAngle = 0
        # LastSampleAngle = 0
        # IntervalSampleAngle = 0
        # IntervalSampleAngle_LastPackage = 0

    def doProcessSimple(self):
        # node [ $distance_q2$, $angle_q6_checkbit$ ]

        nodes = self.scan_node_buf

        all_nodes_counts = self._node_counts
        each_angle = 360.0 / all_nodes_counts

        angle_compensate_nodes = np.zeros((all_nodes_counts, 2), dtype=int)

        for i in range(self.count):
            if nodes[i, 0] != 0:
                angle = (nodes[i, 1] >>
                         Lidar.resp_measurement_angle_shift)/64.0

                inter = int(angle/each_angle)
                angle_pre = angle - inter * each_angle
                angle_next = (inter+1)*each_angle - angle

                if angle_pre < angle_next:
                    if inter < all_nodes_counts:
                        angle_compensate_nodes[inter] = nodes[i]
                else:
                    if inter < all_nodes_counts - 1:
                        angle_compensate_nodes[inter+1] = nodes[i]
                # print(nodes[i], angle, inter, angle_pre, angle_next)
        # print("\n")
        diff_angle = self.scan.config.max_angle - self.scan.config.min_angle
        counts = int(all_nodes_counts * (diff_angle / (2*pi)))
        angle_start = int(pi + self.scan.config.min_angle)
        node_start = int(all_nodes_counts * (angle_start / (2*pi)))

        self.scan.ranges = np.zeros(counts)

        index = 0
        for i in range(all_nodes_counts):
            dist_range = angle_compensate_nodes[i, 0] / 4000

            if i < all_nodes_counts // 2:
                index = all_nodes_counts // 2 - 1 - i
            else:
                index = all_nodes_counts - 1 - (i-all_nodes_counts//2)

            if dist_range > self.scan.config.max_range or dist_range < self.scan.config.min_range:
                dist_range = 0.0

            pos = index - node_start
            if 0 <= pos and pos < counts:
                self.scan.ranges[pos] = dist_range

        if diff_angle == 2*pi:
            self.scan.config.ang_increment = diff_angle / counts
        else:
            self.scan.config.ang_increment = diff_angle / (counts - 1)

        # for i in range(0, self.scan.ranges.shape[0], 3):
            # pass
        # for i in range(20):
        #     angle = self.scan.config.min_angle + i * self.scan.config.ang_increment
        #     dist = self.scan.ranges[i]
        #     print("{}: {}".format(angle, dist))
        # print("\n\n")

    def setDTR(self):
        if not self._isConnect:
            return
        else:
            self._serial.setDTR(1)
            self._serial.flush()

    def clearDTR(self):
        if not self._isConnect:
            return
        else:
            self._serial.setDTR(0)
            self._serial.flush()


# if __name__ == "__main__":
#     lidar = YDLidarX4("/dev/ttyUSB0")
#     lidar.initialize()
#     lidar.startScan()
#     # for i in range(5):
#     # lidar.doProcessSimple()
#     # print(lidar._serial.inWaiting())
#     time.sleep(1)
#     lidar.stopScan()
    # while True:
    #     scan = LaserScan()
