#!/usr/bin/env python3

# Copyright (c) 2022 Lake Fusion Technologies GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import warnings


class SemanticPoint(object):
    """
    Semantic Lidar Point Class

    If the World is set it can query colour information which will be used by Lidar Post-Processing
    """

    _colourCache = {}
    _world = None

    def __init__(self, rawData):
        self.x = float(rawData['x'])

        # flip y for ROS Pointcloud
        self.y = -1.0 * float(rawData['y'])
        self.z = float(rawData['z'])

        self.cosTheta = float(rawData['cosAngle'])
        self.tag = int(rawData['tag'])
        self.valid = True
        self.idx = int(rawData['idx'])

        # if this point belongs to a vehicle, query the colour
        if self.tag == 10:
            self._getColour()
        else:
            self.colour = 255

    @staticmethod
    def setWorld(world):
        SemanticPoint._world = world

    @staticmethod
    def getWorld():
        return SemanticPoint._world

    def getDistance(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def scaleDistance(self, factor):
        self.x *= factor
        self.y *= factor
        self.z *= factor

    def _getColour(self):
        try:
            # try to find the colour in the cache
            self.colour = SemanticPoint._colourCache[self.idx]

        except KeyError:
            if SemanticPoint._world:
                actor = SemanticPoint._world.get_actor(self.idx)

                # is this object an actor?
                if actor is not None:
                    try:
                        # Actor with colour, actor attribute does not contain alpha value
                        rgb = actor.attributes['color'].split(',')

                        # convert colour information to grayscale
                        self.colour = SemanticPoint._colourCache[self.idx] = 0.2126 * float(rgb[0]) + 0.7152 * float(rgb[1]) + 0.0722 * float(rgb[2])
                    except KeyError:
                        # Actor without colour gets white
                        self.colour = SemanticPoint._colourCache[self.idx] = 255
                else:
                    self.colour = SemanticPoint._colourCache[self.idx] = 255
            else:
                warnings.warn("Point world is uninitialised, cannot query actor colour!")
                self.colour = SemanticPoint._colourCache[self.idx] = 255
