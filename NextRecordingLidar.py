#!/usr/bin/env python3

# Copyright (c) 2022 Lake Fusion Technologies GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os.path
from datetime import datetime as dt
from NextLidar import NextLidar

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


class NextRecordingLidar(NextLidar):
    """
    Carla Semantic Lidar Post-Processing class which mimics physical
    characteristics of the real sensor.

    This class also records the processed frames as pcd files.
    """

    _pcdDt = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('intensity', np.float32)])

    def __init__(self, topic, shortRange, sensorActor, rclpy, world, recordingPath):
        super().__init__(topic, shortRange, sensorActor, rclpy, world)

        if len(recordingPath):
            self._recordingDir = os.path.join(recordingPath, dt.now().strftime("%Y%m%d_%H%M%S"), "pcd_" + topic)
            self._createRecordingDir()
            print("Recording into " + str(self._recordingDir))

    def genFilename(self, ts):
        return self._filenamePrefix + str(float(super()._baseTs) + ts)[:19] + '.pcd'

    def writeHeader(self, fileObj):
        pointCount = str(len(self._pcList) // 4)
        header = ["# .PCD v0.7 - Point Cloud Data file format\n",
                  "VERSION 0.7\n",
                  "FIELDS x y z intensity\n",
                  "SIZE 4 4 4 4\n",
                  "TYPE F F F F\n",
                  "COUNT 1 1 1 1\n",
                  "WIDTH " + pointCount,
                  "\nHEIGHT 1\n",
                  "VIEWPOINT 0 0 0 1 0 0 0\n",
                  "POINTS " + pointCount,
                  "\nDATA ascii\n"]

        for line in header:
            fileObj.write(line)

    def _createRecordingDir(self):
        try:
            os.makedirs(self._recordingDir)
            self._recording = True
            # add filename prefix to save time during export
            self._filenamePrefix = os.path.join(self._recordingDir, 'lidar_')

        except OSError as error:
            print(error)
            # do not record if the dir creation fails
            self._recording = False

    def processCloud(self, lidarMeasurement, ts):
        super().processCloud(lidarMeasurement, ts)

        if self._recording:
            fo = open(self.genFilename(ts), 'w')

            self.writeHeader(fo)

            points = np.frombuffer(self._pcList, dtype=self._pcdDt)

            for point in points:
                fo.write(str('{:1.8f} {:1.8f} {:1.8f} {:1.8f}\n').format(*point))
