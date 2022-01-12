#!/usr/bin/env python3

# Copyright (c) 2022 Lake Fusion Technologies GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time
from random import random as random
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math
from SemanticPoint import SemanticPoint

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

gMaxDictSearch = 0
gMaxCarlaQuery = 0
gMaxPPDuration = 0


class NextLidar(object):
    """
    Carla Semantic Lidar Post-Processing class which mimics physical
    characteristics of the real sensor.
    """

    # common lidar parameters
    # max distance noise is 20cm
    _noiseDistance = 0.2

    # assumed bias factor of the sensor
    # 1 - 1 / 1000
    _biasEpsilonFactor = 0.999

    # 1 / 255 * p0Factor
    _lum2p0Factor = 0.00392156862745098

    # default reflectivity is 1 (reflectivity of a white object)
    _p0Default = 1.0

    # Beam divergence: sinus of alpha where alpha assumed to be = 0.1°
    _sinAlpha = 0.00174533

    # speed of light m/s
    _c = 299792458.0

    # pulsewidth: t0 = 20ns in seconds
    _t0 = 0.00000002

    # TPR(d) epsilonRange:
    # a point in range between 0-dMax will always be detected
    # epsilon range defines a range between [dMax, dMax + epsilonRange] in which
    # probability of the point being valid decreases linearly towards reaching dMax + epsilonRange
    # no point will be valid at dMax + epsilonRange
    _probEpsilonRange = 10.0
    _probEpsilonRangeDiv = 1000.0 / _probEpsilonRange

    _nonReflectorIntensityLimit = 0.5
    _vegIntensityLimit = 0.3

    _retroReflectorIntensity = 1.0

    _grassMeanShift = 0.5
    _grassVerticalFactor = 0.5
    _grassHorizontalFactor = 0.1

    _publishFields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

    _dt = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('cosAngle', np.float32), ('idx', np.uint32), ('tag', np.uint32)])

    _baseTs = int(time.time())

    def __init__(self, topic, shortRange, sensorActor, rclpy, world, *_):
        if not SemanticPoint.getWorld():
            SemanticPoint.setWorld(world)

        self.shortRange = shortRange
        self.rosNode = rclpy.create_node('node' + topic)
        self.topic = topic

        # vertical angle of the top row
        self.degTopRow = float(sensorActor.attributes['upper_fov'])
        self.rows = int(sensorActor.attributes['channels'])
        self.degPerRow = (self.degTopRow - float(sensorActor.attributes['lower_fov'])) / float(self.rows)

        # special handling of p0 for the road
        if shortRange:
            # d10: distance in meters which the sensor can detect a 10% target
            self.d10 = 15
            # custom p0 value for asphalt to mimic the characteristic of ibeoNext
            self._p0AsphaltDefault = 50 * NextLidar._lum2p0Factor

        else:
            self.d10 = 71
            # long range looks further than IbeoNext60 on asphalt
            self._p0AsphaltDefault = 30

        self.header = Header(frame_id='map')

        # create a publisher for this lidar
        self.publisher = self.rosNode.create_publisher(PointCloud2, self.topic,
                                                       QoSProfile(depth=1))

    def __del__(self):
        pass

    def _applyInterlacing(self, point):
        # calculate elevation
        elevation = math.degrees(math.atan2(point.z, math.sqrt(point.x * point.x + point.y * point.y)))

        # calculate row
        row = (round(-(elevation - self.degTopRow) / self.degPerRow))

        if ((row < 20) or (row > 80)) and ((row % 2) == 1):
            point.valid = False

    def _addDistanceNoiseAndBias(self, point):
        distance = point.getDistance()

        if distance > 0:
            distWithNoise = distance - (NextLidar._noiseDistance * random())
            point.scaleDistance((distWithNoise / distance) * NextLidar._biasEpsilonFactor)

    def processCloud(self, lidarMeasurement, ts):
        tsSec = int(ts)
        self.header.stamp.sec = tsSec
        self.header.stamp.nanosec = int((ts - tsSec) * 1000000000.0)

        pointList = np.frombuffer(lidarMeasurement.raw_data, dtype=NextLidar._dt)

        cloudSize = pointList.shape[0]

        self._pcList = bytearray()

        self.reducedCloudSize = 0

        for i in range(cloudSize):
            pt = SemanticPoint(pointList[i])

            # apply post processing
            try:
                self.applyPostProcessing(pt)
            # more robustness for unexpected cosTheta and r values
            except ValueError as e:
                pt.valid = False
            except ZeroDivisionError as e:
                pt.valid = False

            if pt.valid:
                self.reducedCloudSize += 1
                self._pcList += struct.pack("f", pt.x)
                self._pcList += struct.pack("f", pt.y)
                self._pcList += struct.pack("f", pt.z)
                self._pcList += struct.pack("f", pt.i)

        self._pc = PointCloud2(header=self.header,
                               height=1,
                               width=self.reducedCloudSize,
                               is_dense=False,
                               is_bigendian=False,
                               fields=NextLidar._publishFields,
                               point_step=16,
                               row_step=16 * self.reducedCloudSize,
                               data=memoryview(self._pcList))

    def publish(self):
        self.publisher.publish(self._pc)

    # filter out points depending on colour, distance, apply intensity effects
    def applyPostProcessing(self, pt):
        global gMaxPPDuration

        start = time.time()

        r = pt.getDistance()

        rSinAlpha = r * NextLidar._sinAlpha
        thetaRad = math.acos(pt.cosTheta)
        tanTheta = math.tan(thetaRad)

        thetaAbove80Deg = thetaRad > 1.396263

        # determine p0
        # if this is a vehicle, use the colour in calculation
        p0 = 0
        dMax = 0
        if pt.tag == 10:
            p0 = pt.colour * NextLidar._lum2p0Factor
        else:
            # if this point belongs to the road
            if pt.tag == 7:
                p0 = self._p0AsphaltDefault
            else:
                p0 = NextLidar._p0Default

        # if the theta is above 80°
        if thetaAbove80Deg:
            dMax = math.sqrt(10.0 * p0 * (NextLidar._t0 / (tanTheta * rSinAlpha )) * NextLidar._c * pt.cosTheta) * self.d10
        else:
            dMax = math.sqrt(10.0 * p0 * pt.cosTheta) * self.d10

        # invalidate point depending on its reflectivity
        currentEpsilon = r - dMax

        # check if the point distance is > dMax
        if currentEpsilon > 0.0:
            # check if the point distance is < dMax + epsilonRange
            if currentEpsilon < NextLidar._probEpsilonRange:
                if (random() * 1000) < (currentEpsilon * NextLidar._probEpsilonRangeDiv):
                    pt.valid = False
            else:
                pt.valid = False

        # if the point has not been filtered out apply further post processing effects
        if pt.valid:
            # calculate intensity depending on the angle of incidence (saturated at nonReflectorIntensityLimit)
            pt.i = min(5.0 * p0 * (pt.cosTheta / r), NextLidar._nonReflectorIntensityLimit)

            # if the colour is not black and intensity is below minimum observed, apply lower limit
            if (p0 > 0) and (pt.i < 0.04):
                pt.i = 0.04

            # if the instance is a short range, apply interlacing on upper and lower rows
            if self.shortRange:
                self._applyInterlacing(pt)

            self._addDistanceNoiseAndBias(pt)

            # if this point belongs to a terrain, emulate grass structure
            # and limit intensity to the nominal value
            if pt.tag == 22:
                if pt.i > NextLidar._vegIntensityLimit:
                    pt.i = NextLidar._vegIntensityLimit
                self._emulateGrass(pt)
            else:
                # if this point belongs to vegetation
                if pt.tag == 9:
                    if pt.i > NextLidar._vegIntensityLimit:
                        pt.i = NextLidar._vegIntensityLimit
                else:
                    # if this is a traffic sign, assign the intensity value for retroreflection
                    if pt.tag == 12:
                        pt.i = NextLidar._retroReflectorIntensity

        ppDuration = time.time() - start
        if ppDuration > gMaxPPDuration:
            gMaxPPDuration = ppDuration
            print("Max Post Processing Duration: " + str(gMaxPPDuration * 1000.0) + " ms")

    def _emulateGrass(self, pt):
        grassNoise = random() - self._grassMeanShift
        pt.x += grassNoise * self._grassHorizontalFactor
        pt.y += grassNoise * self._grassHorizontalFactor
        pt.z += (grassNoise * self._grassVerticalFactor) + self._grassMeanShift
